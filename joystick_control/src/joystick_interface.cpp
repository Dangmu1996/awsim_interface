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
    bool hazard_indicator = false;
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
            haz_pub_ = this->create_publisher<HazardLightsCommand>("/control/command/hazard_lights_cmd", durable_qos);
            timer_ = this->create_wall_timer(100ms, std::bind(&Joystick::timerCB, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<HazardLightsCommand>::SharedPtr haz_pub_;
        // rclcpp::QoS durable_qos{1};

        Command cmd_;

        void joyCB(const Joy &msg)
        {
            if(msg.buttons[1] == 1)
                cmd_.hazard_indicator = true;
            else
                cmd_.hazard_indicator = false;
        }

        void timerCB()
        {
            auto hz_light = HazardLightsCommand();
            if(cmd_.hazard_indicator)
                hz_light.command=HazardLightsCommand::ENABLE;
            else
                hz_light.command=HazardLightsCommand::DISABLE;

            haz_pub_->publish(hz_light);
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick>());
    rclcpp::shutdown();
    return 0;
}