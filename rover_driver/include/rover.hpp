#pragma once

#include <chrono>
#include <data.hpp>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rover_msgs/msg/raw_command.hpp"
#include "rover_msgs/msg/raw_data.hpp"
#include "rover_msgs/msg/raw_motor_command.hpp"
#include "pi_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "timestamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

using duration = std::chrono::nanoseconds;
namespace rover
{
/// This node supervises a Connection node and translates between low-level
/// commands and high-level commands.
class Rover : public rclcpp::Node
{
public:
  Rover();

protected:
  /// If no cmd_vel is recieved, reset the pi controllers
  duration reset_pi_controller_timeout = 1000ms;

  std::unordered_map<uint8_t, std::shared_ptr<const Timestamped<std::array<uint8_t, 2>>>>
    most_recent_data;

  /// Speed (m/s) this rover will attain running its motors at full power
  /// forward.
  double top_speed_linear;
  /// Rotational speed (rad/s) this rover will attain pushing its motors at full
  /// efford in opposite directions
  double top_speed_angular;

  /// Describes the relation between the encoder frequencies and the rover
  /// velocity
  Eigen::Matrix2d encoder_frequency_lr_to_twist_fl;

  std::string odom_frame_id;
  std::string odom_child_frame_id;
  float odom_pose_x;
  float odom_pose_y;
  float odom_orientation_z;
  geometry_msgs::msg::TransformStamped tf;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  bool publish_tf;

  std::unique_ptr<PIController> left_motor_controller;
  std::unique_ptr<PIController> right_motor_controller;

  bool left_wheel_fwd{};
  bool right_wheel_fwd{};

  /// Callback for velocity commands
  void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  /// Callback for new raw data received
  void on_raw_data(rover_msgs::msg::RawData::ConstSharedPtr data);

  void update_firmware_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void update_power_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void update_drive_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void on_reset_pi_controllers();

  std::shared_ptr<diagnostic_updater::Updater> updater;

  rclcpp::Time odom_last_time;
  data::LeftMotorEncoderState::Value odom_last_encoder_position_left;
  data::RightMotorEncoderState::Value odom_last_encoder_position_right;
  rclcpp::TimerBase::SharedPtr tmr_odometry;
  void update_odom();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  /// Subscription for raw data coming from the rover
  rclcpp::Subscription<rover_msgs::msg::RawData>::SharedPtr sub_raw_data;
  /// Publisher for efforts going to the rover
  rclcpp::Publisher<rover_msgs::msg::RawMotorCommand>::SharedPtr pub_motor_efforts;
  rclcpp::Publisher<rover_msgs::msg::RawCommand>::SharedPtr pub_rover_command;
  rclcpp::TimerBase::SharedPtr reset_pi_controller_timer;

  template <typename T>
  std::unique_ptr<Timestamped<typename T::Value>> get_recent()
  {
    try {
      auto raw = *most_recent_data.at(T::which());
      auto value = T::decode(raw.state);
      return std::make_unique<Timestamped<typename T::Value>>(raw.time, value);
    } catch (std::out_of_range &) {
      return nullptr;
    }
  }
};
}  // namespace rover
