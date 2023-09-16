#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"

#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include <iostream>

#include "joystick_control/vehicle_cmd_filter.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using sensor_msgs::msg::Joy;

using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::VelocityReport;


struct Command
{
    AckermannControlCommand control;
    HazardLightsCommand hazard_light;
    TurnIndicatorsCommand turn_light_indicator;
    GearCommand gear_cmd;
};


class Joystick : public rclcpp::Node
{
    public:
        Joystick()
        : Node("joystick_interface")
        {
            rclcpp::QoS durable_qos{1};
            durable_qos.transient_local();
            joy_sub_ = this->create_subscription<Joy>("joy", 10, std::bind(&Joystick::joyCB, this, _1));
            gear_sub_ = this->create_subscription<GearReport>("/vehicle/status/gear_status", 10, std::bind(&Joystick::gearCB, this, _1));
            vel_sub_ = this->create_subscription<VelocityReport>("/vehicle/status/velocity_status", 10 , std::bind(&Joystick::velCB, this, _1));

            hazard_light_pub_ = this->create_publisher<HazardLightsCommand>("/control/command/hazard_lights_cmd", durable_qos);
            turn_indi_pub_ = this->create_publisher<TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", durable_qos);
            
            gear_pub_ = this->create_publisher<GearCommand>("/control/command/gear_cmd", durable_qos);
            cmd_.gear_cmd.command=GearCommand::PARK;

            cmd_pub_ = this->create_publisher<AckermannControlCommand>("/control/command/control_cmd", durable_qos);

            timer_ = this->create_wall_timer(100ms, std::bind(&Joystick::timerCB, this));
            cmd_timer_ = this->create_wall_timer( 20ms, std::bind(&Joystick::cmdTimerCB, this));

            cmd_filter_.setWheelBase(2.5);
            cmd_filter_.setVelLim(50.0);
            cmd_filter_.setLonAccLim(2.0);
            cmd_filter_.setLonJerkLim(0.5);
            cmd_filter_.setLatAccLim(2.0);
            cmd_filter_.setLatJerkLim(7.0);
            cmd_filter_.setActualSteerDiffLim(1.0);        
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr cmd_timer_;
        rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<GearReport>::SharedPtr gear_sub_;
        rclcpp::Subscription<VelocityReport>::SharedPtr vel_sub_;

        rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_light_pub_;
        rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_indi_pub_;
        rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
        rclcpp::Publisher<AckermannControlCommand>::SharedPtr cmd_pub_;

        VehicleCmdFilter cmd_filter_;
        double accel_, brake_, steer_angle_;

        Command cmd_;
        std::shared_ptr<rclcpp::Time> prev_time_;

        float cur_vel_;

        void joyCB(const Joy &msg)
        {
            //Setting Light
            if(msg.buttons[4] == 1 && msg.buttons[5] == 0)
            {
                cmd_.hazard_light.command=HazardLightsCommand::DISABLE;
                cmd_.turn_light_indicator.command=TurnIndicatorsCommand::ENABLE_LEFT;
            }
            else if(msg.buttons[4]==0 && msg.buttons[5] == 1)
            {
                cmd_.hazard_light.command=HazardLightsCommand::DISABLE;
                cmd_.turn_light_indicator.command=TurnIndicatorsCommand::ENABLE_RIGHT;
            }
            else if(msg.buttons[4]==0 && msg.buttons[5] == 0)
            {
                cmd_.hazard_light.command=HazardLightsCommand::DISABLE;
                cmd_.turn_light_indicator.command=TurnIndicatorsCommand::DISABLE;
            }
            else
            {
                cmd_.hazard_light.command=HazardLightsCommand::ENABLE;
                cmd_.turn_light_indicator.command=TurnIndicatorsCommand::DISABLE;
            }

            //Setting Gear
            if(msg.buttons[1] == 1)
                cmd_.gear_cmd.command=GearCommand::DRIVE;
            if(msg.buttons[0] ==1)
                cmd_.gear_cmd.command=GearCommand::PARK;
            if(msg.buttons[2]==1)
                cmd_.gear_cmd.command=GearCommand::REVERSE;

            //Setting Accel and Brake
            accel_ = (1.0 - msg.axes[2])*2;
            brake_ = (1.0 - msg.axes[5])*5;
            steer_angle_ = msg.axes[3]*0.52;
        }

        void gearCB(const GearReport &msg)
        {
            RCLCPP_INFO(this->get_logger(), "Gear: '%d'", msg.report);
        }

        void velCB(const VelocityReport &msg)
        {
            cur_vel_= msg.longitudinal_velocity;
        }

        void timerCB()
        {
            hazard_light_pub_->publish(cmd_.hazard_light);
            turn_indi_pub_->publish(cmd_.turn_light_indicator);
            gear_pub_->publish(cmd_.gear_cmd);
        }

        void cmdTimerCB()
        {
            if(cmd_.gear_cmd.command == GearCommand::PARK)
                return;
            
            // if(cmd_.gear_cmd.command == GearCommand::REVERSE)
            //     return;
            
            // double dt= getDt();
            if(cmd_.gear_cmd.command == GearCommand::DRIVE)
            {
                cmd_.control.longitudinal.acceleration = accel_ - brake_;
                cmd_.control.lateral.steering_tire_angle = steer_angle_;
            }
            else if(cmd_.gear_cmd.command == GearCommand::REVERSE)
            {
                cmd_.control.longitudinal.acceleration = -(accel_ - brake_);
                cmd_.control.lateral.steering_tire_angle = steer_angle_;
            }
            
            
            
            // AckermannControlCommand filter_command = filterControlCommand(cmd_.control);
            cmd_pub_->publish(cmd_.control);

        }

        double getDt()
        {
            if (!prev_time_) 
            {
                prev_time_ = std::make_shared<rclcpp::Time>(this->now());
                return 0.0;
            }

            const auto current_time = this->now();
            const auto dt = (current_time - *prev_time_).seconds();
            *prev_time_ = current_time;

            return dt;
        }

        AckermannControlCommand filterControlCommand(const AckermannControlCommand & in)
        {
          AckermannControlCommand out = in;
          const double dt = getDt();

          cmd_filter_.filterAll(dt, steer_angle_, out);
     
          auto prev_values = getActualStatusAsCommand();

          prev_values.longitudinal = out.longitudinal;

          cmd_filter_.setPrevCmd(prev_values);
          

          return out;
        }

        AckermannControlCommand getActualStatusAsCommand()
        {
            AckermannControlCommand status;
            status.stamp = status.lateral.stamp = status.longitudinal.stamp = this->now();
            status.lateral.steering_tire_angle = steer_angle_;
            status.lateral.steering_tire_rotation_rate = 0.0;
            status.longitudinal.speed = cur_vel_;;
            status.longitudinal.acceleration = accel_-brake_;
            return status;
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick>());
    rclcpp::shutdown();
    return 0;
}