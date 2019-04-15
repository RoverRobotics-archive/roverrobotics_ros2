#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "openrover_core_msgs/msg/raw_motor_command.hpp"
namespace openrover
{
/// This node supervises a Connection node and translates between low-level commands and high-level commands.
class Rover : public rclcpp::Node
{
public:
  Rover();

protected:
  void on_velocity(geometry_msgs::msg::Twist::SharedPtr);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  rclcpp::Publisher<openrover_core_msgs::msg::RawMotorCommand>::SharedPtr pub_motor_efforts;
};
}  // namespace openrover