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

using std::placeholders::_1;
using namespace std::chrono_literals;

using sensor_msgs::msg::Joy;

using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;


struct Command
{
    HazardLightsCommand hazard_light;
    TurnIndicatorsCommand turn_light_indicator;
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

            hazard_light_pub_ = this->create_publisher<HazardLightsCommand>("/control/command/hazard_lights_cmd", durable_qos);
            turn_indi_pub_ = this->create_publisher<TurnIndicatorsCommand>("/control/command/turn_indicators_cmd", durable_qos);

            timer_ = this->create_wall_timer(100ms, std::bind(&Joystick::timerCB, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<Joy>::SharedPtr joy_sub_;

        rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_light_pub_;
        rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_indi_pub_;
        // rclcpp::QoS durable_qos{1};

        Command cmd_;

        void joyCB(const Joy &msg)
        {
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
        }

        void timerCB()
        {
            hazard_light_pub_->publish(cmd_.hazard_light);
            turn_indi_pub_->publish(cmd_.turn_light_indicator);
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick>());
    rclcpp::shutdown();
    return 0;
}