#include "rover.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <Eigen/src/Core/Matrix.h>
#include "data.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/node_options.hpp"

using std::placeholders::_1;
using namespace openrover;
using namespace std::literals::chrono_literals;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using namespace openrover_core_msgs;

Rover::Rover() : Node("rover", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Starting rover driver node");

  sub_raw_data = create_subscription<msg::RawData>("raw_data", rclcpp::QoS(32),
                                                   [=](msg::RawData::ConstSharedPtr msg) { on_raw_data(msg); });
  double diagnostics_frequency = declare_parameter("diagnostics_frequency", 0.2);
  tmr_diagnostics = create_wall_timer(1s / diagnostics_frequency, [=]() { update_diagnostics(); });
  double odometry_frequency = declare_parameter("odometry_frequency", 10.0);
  tmr_odometry = create_wall_timer(1s / odometry_frequency, [=]() { update_odom(); });

  sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(1),
                                                               [=](Twist::ConstSharedPtr msg) { on_cmd_vel(msg); });
  pub_rover_command = create_publisher<openrover_core_msgs::msg::RawCommand>("openrover_command", rclcpp::QoS(32));
  pub_motor_efforts = create_publisher<openrover_core_msgs::msg::RawMotorCommand>("motor_efforts", rclcpp::QoS(1));
  pub_diagnostics = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(1));
  pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom_raw", rclcpp::QoS(4));

  // based on the physical capabilities of the rover. Depends on the wheel configuration (2wd/4wd/treads) and terrain
  top_speed_linear = declare_parameter("top_speed_linear", 3.05);
  top_speed_angular = declare_parameter("top_speed_angular", 16.2);

  /// If both motor encoders have traveled a combined n increments, this times n is how far the rover has traveled.
  // this value determined by driving straight and dividing distance by average encoder reading
  auto meters_per_encoder_sum = declare_parameter("meters_per_encoder_sum", 0.0006875);
  /// If the left motor encoder has traveled n increments more than the right motor encoder,
  /// this times n is how far the rover has rotated
  /// this value determined by driving in a circle and dividing rotation by difference in encoder readings
  auto radians_per_encoder_difference = declare_parameter("radians_per_encoder_difference", 0.00371);

  odom_frame_id = declare_parameter("odom_frame_id", "odom");
  odom_child_frame_id = declare_parameter("odom_child_frame_id", "base_footprint");

  encoder_frequency_lr_to_twist_fl <<  //
      meters_per_encoder_sum,
      meters_per_encoder_sum,  //
      -radians_per_encoder_difference, +radians_per_encoder_difference;

  auto pi_p = declare_parameter("motor_control_gain_p", 0.0001);
  auto pi_i = declare_parameter("motor_control_gain_i", 0.01);
  auto pi_windup = declare_parameter("motor_control_windup", 10);
  auto now = get_clock()->now();

  left_motor_controller = std::make_unique<PIController>(pi_p, pi_i, pi_windup, now);
  right_motor_controller = std::make_unique<PIController>(pi_p, pi_i, pi_windup, now);
}

/// Takes a number between -1.0 and +1.0 and converts it to the nearest motor command value.
/// values out of range will be clamped to the nearest value in range
uint8_t to_motor_command(double d)
{
  if (std::isnan(d))
  {
    return 125;
  }
  d = trunc(d * 125);
  d = std::min(d, +125.0);
  d = std::max(d, -125.0);
  d += 125;
  return static_cast<uint8_t>(d);
}

void Rover::on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  // expecting values in the range of +/- linear_top_speed and +/- angular top speed
  auto linear_rate = msg->linear.x;
  if (top_speed_linear < std::abs(linear_rate))
  {
    RCLCPP_WARN(get_logger(), "Requested linear velocity %f higher than maximum %f", linear_rate, top_speed_linear);
  }

  auto turn_rate = msg->angular.z;
  if (top_speed_angular < std::abs(turn_rate))
  {
    RCLCPP_WARN(get_logger(), "Requested angular velocity %f higher than maximum %f", turn_rate, top_speed_angular);
  }

  Eigen::Vector2d twist_fl(linear_rate, turn_rate);

  RCLCPP_DEBUG(get_logger(), "target velocity = fwd:%f ccw:%f", linear_rate, turn_rate);
  auto encoder_target_freqs = encoder_frequency_lr_to_twist_fl.inverse() * twist_fl;

  auto l_motor = encoder_target_freqs[0];
  auto r_motor = encoder_target_freqs[1];

  // save off wheel direction for odometry purposes
  left_wheel_fwd = (l_motor >= 0);
  right_wheel_fwd = (r_motor >= 0);

  left_motor_controller->set_target(l_motor);
  right_motor_controller->set_target(r_motor);

  RCLCPP_DEBUG(get_logger(), "Updated target motor speeds %f %f", l_motor, r_motor);
}

void openrover::Rover::update_diagnostics()
{
  // Ask the rover to send the next batch of diagnostics data
  const std::vector<uint8_t> DIAGNOSTIC_DATA_ELEMENTS{
    data::MotorTemperature1::which(),     data::LeftMotorStatus::which(),      data::RightMotorStatus::which(),
    data::FlipperMotorStatus::which(),    data::BatteryChargingState::which(), data::BatteryAStateOfCharge::which(),
    data::BatteryBStateOfCharge::which(),
  };
  for (auto which : DIAGNOSTIC_DATA_ELEMENTS)
  {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = which;
    pub_rover_command->publish(cmd);
  }

  auto diagnostics = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  diagnostics->header.stamp = get_clock()->now();
  {
    diagnostic_msgs::msg::DiagnosticStatus rover_status;
    rover_status.hardware_id = "openrover";
    if (auto v = get_recent<data::RoverVersion>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "firmware revision";
      kv.value = v->state.to_string();
      rover_status.values.push_back(kv);
    }
    diagnostics->status.push_back(rover_status);
  }
  {
    diagnostic_msgs::msg::DiagnosticStatus encoders_status;
    encoders_status.hardware_id = "openrover/motor encoders";
    if (auto data = get_recent<data::LeftMotorEncoderState>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "left position";
      kv.value = std::to_string(data->state);
      encoders_status.values.push_back(kv);
    }
    if (auto data = get_recent<data::RightMotorEncoderState>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "right position";
      kv.value = std::to_string(data->state);
      encoders_status.values.push_back(kv);
    }
    if (auto data = get_recent<data::LeftMotorEncoderPeriod>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "left period";
      kv.value = std::to_string(data->state);
      encoders_status.values.push_back(kv);
    }
    if (auto data = get_recent<data::RightMotorEncoderPeriod>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "right period";
      kv.value = std::to_string(data->state);
      encoders_status.values.push_back(kv);
    }
    diagnostics->status.push_back(encoders_status);
  }
  {
    diagnostic_msgs::msg::DiagnosticStatus motors_status;
    motors_status.hardware_id = "openrover/motors";
    if (auto data = get_recent<data::MotorTemperature1>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "temperature";
      kv.value = std::to_string(data->state);
      motors_status.values.push_back(kv);
    }
    diagnostics->status.push_back(motors_status);
  }
  {
    diagnostic_msgs::msg::DiagnosticStatus power_status;
    power_status.hardware_id = "openrover/power";
    if (auto data = get_recent<data::BatteryAStateOfCharge>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "battery A state of charge";
      kv.value = std::to_string(data->state);
      power_status.values.push_back(kv);
    }
    if (auto data = get_recent<data::BatteryBStateOfCharge>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "battery B state of charge";
      kv.value = std::to_string(data->state);
      power_status.values.push_back(kv);
    }
    if (auto data = get_recent<data::BatteryChargingState>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "battery charging state";
      kv.value = std::to_string(data->state);
      power_status.values.push_back(kv);
    }
    diagnostics->status.push_back(power_status);
  }

  pub_diagnostics->publish(std::move(diagnostics));
}

void openrover::Rover::update_odom()
{
  for (auto arg : { data::LeftMotorEncoderState::Which, data::RightMotorEncoderState::Which,
                    data::LeftMotorEncoderPeriod::Which, data::RightMotorEncoderPeriod::Which })
  {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = arg;
    pub_rover_command->publish(cmd);
  }

  auto now = get_clock()->now();
  auto left_encoder_position = get_recent<data::LeftMotorEncoderState>();
  auto right_encoder_position = get_recent<data::RightMotorEncoderState>();
  auto left_period = get_recent<data::LeftMotorEncoderPeriod>();
  auto right_period = get_recent<data::RightMotorEncoderPeriod>();

  if (!left_encoder_position || !right_encoder_position || !left_period || !right_period)
  {
    RCLCPP_WARN_SKIPFIRST(get_logger(), "Odometry not ready yet");
    return;
  }

  if (odom_last_time.get_clock_type() != RCL_ROS_TIME)
  {
    RCLCPP_INFO(get_logger(), "Initializing odometry");
    odom_last_encoder_position_left = left_encoder_position->state;
    odom_last_encoder_position_right = right_encoder_position->state;
    odom_last_time = now;

    return;
  }

  if (left_encoder_position->time < odom_last_time || right_encoder_position->time < odom_last_time ||
      left_period->time < odom_last_time || right_period->time < odom_last_time)
  {
    RCLCPP_WARN(get_logger(), "Trying to compute odometry based on stale data");
  }

  // encoder displacement since last time we did odometry.
  auto dt = (now - odom_last_time).seconds();

  // Determine encoder frequency.
  // earlier versions of the firmware don't return the encoder position so we have to make do with the period estimate
  Eigen::Vector2d encoder_frequency_lr;
  if (left_encoder_position->state == 0 && right_encoder_position->state == 0)
  {
    encoder_frequency_lr = { ((left_period->state == 0) ? 0 : 1.0 / (left_period->state)),
                             ((right_period->state == 0) ? 0 : 1.0 / (right_period->state)) };
    if (!left_wheel_fwd)
      encoder_frequency_lr[0] *= -1;
    if (!right_wheel_fwd)
      encoder_frequency_lr[1] *= -1;
    // ^ the encoder doesn't actually have the wheel direction. So we fake it by assuming the same direction as the last
    // commanded direction we gave to the wheel
  }
  else
  {
    encoder_frequency_lr = { (left_encoder_position->state - odom_last_encoder_position_left) / dt,
                             (right_encoder_position->state - odom_last_encoder_position_right) / dt };
    // ^ remember these values are signed. But taking the difference a-b as signed ints will give either a-b or 1<<16 -
    // a-b, whichever has the lower absolute value. This is exactly what we want.
  }

  // drive the motors based on closed-loop control
  {
    auto l_effort = left_motor_controller->step(now, encoder_frequency_lr[0]);
    auto r_effort = right_motor_controller->step(now, encoder_frequency_lr[1]);

    openrover_core_msgs::msg::RawMotorCommand e;
    e.left = to_motor_command(l_effort);
    e.right = to_motor_command(r_effort);
    e.flipper = to_motor_command(0);  // todo
    pub_motor_efforts->publish(e);
  }

  // determine the rover's current velocity and publish for state estimation
  Eigen::Vector2d encoder_frequency_lr_variance;
  encoder_frequency_lr_variance = (0.1 * encoder_frequency_lr).array().square();
  auto twist = encoder_frequency_lr_to_twist_fl * encoder_frequency_lr;
  auto twist_covariance = encoder_frequency_lr_to_twist_fl * encoder_frequency_lr_variance.asDiagonal() *
                          encoder_frequency_lr_to_twist_fl.adjoint();
  {
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = odom_frame_id;
    odom->child_frame_id = odom_child_frame_id;
    odom->header.stamp = now;

    // In the odom_frame_id
    odom->pose.covariance.fill(-1.0);
    // We don't have any odom pose, but rviz complains if the Quat is not normalized
    odom->pose.pose.orientation.z = 1.0;

    // In the odom_child_frame_id
    odom->twist.twist.linear.x = twist[0];
    odom->twist.twist.angular.z = twist[1];

    odom->twist.covariance.fill(0.0);
    odom->twist.covariance[0 + 0 * 6] = twist_covariance(0, 0);
    odom->twist.covariance[0 + 5 * 6] = odom->twist.covariance[5 + 0 * 6] = twist_covariance(0, 1);
    odom->twist.covariance[5 + 5 * 6] = twist_covariance(1, 1);

    pub_odom->publish(std::move(odom));
  }

  odom_last_encoder_position_left = left_encoder_position->state;
  odom_last_encoder_position_right = right_encoder_position->state;
  odom_last_time = now;
}

void openrover::Rover::on_raw_data(openrover_core_msgs::msg::RawData::ConstSharedPtr data)
{
  most_recent_data[data->which] = std::make_unique<Timestamped<data::RawValue>>(get_clock()->now(), data->value);
}