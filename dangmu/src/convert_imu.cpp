#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class ConvertImu : public rclcpp::Node
{
    public:
        ConvertImu()
        : Node("convert_imu")
        {
            subscription_=this->create_subscription<sensor_msgs::msg::Imu>(
            "sensing/imu/tamagawa/imu_raw",rclcpp::SensorDataQoS() ,std::bind(&ConvertImu::imu_callback, this, _1));

            publisher_=this->create_publisher<sensor_msgs::msg::Imu>("dangmu/imu_raw", rclcpp::SensorDataQoS());
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

        bool init_flag_ = false;
        sensor_msgs::msg::Imu data_;

        void imu_callback(const sensor_msgs::msg::Imu & msg)
        {
            if(!init_flag_){
                    data_.header.frame_id= msg.header.frame_id;
            }
            
            data_.header.stamp=this->get_clock()->now();
            data_.orientation=msg.orientation;
            data_.orientation_covariance=msg.orientation_covariance;
            data_.angular_velocity=msg.angular_velocity;
            data_.angular_velocity_covariance=msg.angular_velocity_covariance;
            data_.linear_acceleration.x=msg.linear_acceleration.x;
            data_.linear_acceleration.y=msg.linear_acceleration.y;
            data_.linear_acceleration.z=std::round(msg.linear_acceleration.z*1000.0+0.5)/1000.0;
            data_.linear_acceleration_covariance=msg.linear_acceleration_covariance;
        
            publisher_->publish(data_);
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvertImu>());
    rclcpp::shutdown();
    return 0;
}
