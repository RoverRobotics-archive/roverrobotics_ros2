#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "openrover_core_msgs/msg/raw_motor_command.hpp"

/// This node supervises an OpenRoverSerial node and translates between low-level commands and high-level commands.
class OpenRoverNode : public rclcpp::Node {
public:
	OpenRoverNode();
protected:
	void on_velocity(geometry_msgs::msg::Twist::SharedPtr);

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
	rclcpp::Publisher<openrover_core_msgs::msg::RawMotorCommand>::SharedPtr pub_motor_efforts;
};