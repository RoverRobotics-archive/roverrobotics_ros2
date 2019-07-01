#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "openrover_core_msgs/msg/raw_motor_command.hpp"
#include "openrover_core_msgs/msg/raw_data.hpp"
#include "openrover_core_msgs/msg/raw_command.hpp"
#include "rclcpp/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <data.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "timestamped.hpp"
#include "pi_controller.hpp"
#include "eigen3/Eigen/Dense"

namespace openrover
{
/// This node supervises a Connection node and translates between low-level commands and high-level commands.
class Rover : public rclcpp::Node
{
public:
  Rover();

protected:
  std::unordered_map<uint8_t, std::shared_ptr<const Timestamped<std::array<uint8_t, 2>>>> most_recent_data;

  /// Speed (m/s) this rover will attain running its motors at full power forward.
  double top_speed_linear;
  /// Rotational speed (rad/s) this rover will attain pushing its motors at full efford in opposite directions
  double top_speed_angular;

  /// Describes the relation between the encoder frequencies and the rover velocity
  Eigen::Matrix2d encoder_frequency_lr_to_twist_fl;

  std::string odom_frame_id;
  std::string odom_child_frame_id;

  std::unique_ptr<PIController> left_motor_controller;
  std::unique_ptr<PIController> right_motor_controller;

  bool left_wheel_fwd{};
  bool right_wheel_fwd{};

  /// Callback for velocity commands
  void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /// Callback for new raw data received
  void on_raw_data(openrover_core_msgs::msg::RawData::ConstSharedPtr data);
  void update_diagnostics();

  rclcpp::TimerBase::SharedPtr tmr_diagnostics;

  rclcpp::Time odom_last_time;
  data::LeftMotorEncoderState::Value odom_last_encoder_position_left;
  data::RightMotorEncoderState::Value odom_last_encoder_position_right;
  std::shared_ptr<std::vector<diagnostic_msgs::msg::KeyValue>> pending_diagnostics;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics;

  rclcpp::TimerBase::SharedPtr tmr_odometry;
  void update_odom();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  /// Subscription for raw data coming from the rover
  rclcpp::Subscription<openrover_core_msgs::msg::RawData>::SharedPtr sub_raw_data;
  /// Publisher for efforts going to the rover
  rclcpp::Publisher<openrover_core_msgs::msg::RawMotorCommand>::SharedPtr pub_motor_efforts;
  rclcpp::Publisher<openrover_core_msgs::msg::RawCommand>::SharedPtr pub_rover_command;

  template <typename T>
  std::unique_ptr<Timestamped<typename T::Value>> get_recent()
  {
    try
    {
      auto raw = *most_recent_data.at(T::which());
      auto value = T::decode(raw.state);
      return std::make_unique<Timestamped<typename T::Value>>(raw.time, value);
    }
    catch (std::out_of_range&)
    {
      return nullptr;
    }
  }
};
}  // namespace openrover
