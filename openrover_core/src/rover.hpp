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

struct EncoderSnapshot
{
  /// The time this snapshot was taken
  rclcpp::Time time;
  /// The signed number of times the tachometer has ticked. This is proportional to the number of wheel rotations,
  /// with a proportionality constant determined by the number of motor windings and the gear ratio.
  /// it may also underflow or overflow, so be careful!
  int16_t state;
};

template <typename T>
struct Timestamped
{
  using State = T;
  Timestamped(rclcpp::Time t, T state) : time(t), state(state){};
  const rclcpp::Time time;
  const T state;
};

struct EncodersState
{
  Timestamped<int16_t> left_motor_prev;
  Timestamped<int16_t> right_motor_prev;
};

namespace openrover
{
/// This node supervises a Connection node and translates between low-level commands and high-level commands.
class Rover : public rclcpp::Node
{
public:
  Rover();

protected:
  std::unordered_map<decltype(openrover_core_msgs::msg::RawData::which),
                     Timestamped<decltype(openrover_core_msgs::msg::RawData::value)>>
      most_recent_data;

  /// Speed (m/s) this rover will attain running its motors at full power forward.
  double top_speed_linear;
  /// Rotational speed (rad/s) this rover will attain pushing its motors at full efford in opposite directions
  double top_speed_angular;

  /// If both motor encoders have traveled n increments, this is how far the rover has traveled.
  double meters_per_encoder_unit;
  /// If the left motor encoder has traveled n increments more than the right motor encoder,
  /// this value times n is how far the rover has rotated
  double radians_per_delta_encoder_unit;

  template <typename T>
  std::shared_ptr<Timestamped<T>> get_recent()
  {
    try
    {
      auto raw = this->most_recent_data.at(T::Which);
      return std::make_shared<Timestamped<T>>(raw.time, T(raw.state));
    }
    catch (std::out_of_range)
    {
      return nullptr;
    }
  }

  /// Callback for velocity commands
  void on_velocity(geometry_msgs::msg::Twist::SharedPtr);
  /// Callback for new raw data received
  void on_raw_data(openrover_core_msgs::msg::RawData::SharedPtr data);

  rclcpp::TimerBase::SharedPtr tmr_diagnostics;
  void update_diagnostics();
  std::shared_ptr<std::vector<diagnostic_msgs::msg::KeyValue>> pending_diagnostics;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics;

  rclcpp::TimerBase::SharedPtr tmr_odometry;
  void update_odometry();

  std::shared_ptr<Timestamped<data::LeftMotorEncoderState>> left_encoder_published;
  std::shared_ptr<Timestamped<data::RightMotorEncoderState>> right_encoder_published;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_obs_vel;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  /// Subscription for raw data coming from the rover
  rclcpp::Subscription<openrover_core_msgs::msg::RawData>::SharedPtr sub_raw_data;
  /// Publisher for efforts going to the rover
  rclcpp::Publisher<openrover_core_msgs::msg::RawMotorCommand>::SharedPtr pub_motor_efforts;
  rclcpp::Publisher<openrover_core_msgs::msg::RawCommand>::SharedPtr pub_rover_command;
};
}  // namespace openrover