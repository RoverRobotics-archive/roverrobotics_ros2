#include "rover.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include "Eigen/src/Core/Matrix.h"
#include "data.hpp"
#include "rclcpp/node_options.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;
using namespace openrover;
using namespace std::literals::chrono_literals;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using namespace openrover_core_msgs;

Rover::Rover() : Node("rover", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Starting rover driver node");

  sub_raw_data = create_subscription<msg::RawData>(
    "raw_data", rclcpp::QoS(32), [=](msg::RawData::ConstSharedPtr msg) { on_raw_data(msg); });

  double odometry_frequency = declare_parameter("odometry_frequency", 10.0);
  tf_broadcast = declare_parameter("broadcast_tf", true);
  tmr_odometry = create_wall_timer(1s / odometry_frequency, [=]() { update_odom(); });

  sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1), [=](Twist::ConstSharedPtr msg) { on_cmd_vel(msg); });
  pub_rover_command =
    create_publisher<openrover_core_msgs::msg::RawCommand>("openrover_command", rclcpp::QoS(32));
  pub_motor_efforts =
    create_publisher<openrover_core_msgs::msg::RawMotorCommand>("motor_efforts", rclcpp::QoS(1));
  pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom_raw", rclcpp::QoS(4));

  double diagnostics_frequency = declare_parameter("diagnostics_frequency", 0.2);
  updater = std::make_shared<diagnostic_updater::Updater>(
    create_sub_node("diagnostic_updater"), 1.0 / diagnostics_frequency);
  updater->setHardwareID("OpenRover");
  updater->add("power", [this](auto & t) { update_power_diagnostics(t); });
  updater->add("firmware", [this](auto & t) { update_firmware_diagnostics(t); });
  updater->add("drive", [this](auto & t) { update_drive_diagnostics(t); });

  // based on the physical capabilities of the rover. Depends on the wheel
  // configuration (2wd/4wd/treads) and terrain
  top_speed_linear = declare_parameter("top_speed_linear", 3.05);
  top_speed_angular = declare_parameter("top_speed_angular", 16.2);

  /// If both motor encoders have traveled a combined n increments, this times n
  /// is how far the rover has traveled.
  // this value determined by driving straight and dividing distance by average
  // encoder reading
  auto meters_per_encoder_sum = declare_parameter("meters_per_encoder_sum", 0.0006875);
  /// If the left motor encoder has traveled n increments more than the right
  /// motor encoder, this times n is how far the rover has rotated this value
  /// determined by driving in a circle and dividing rotation by difference in
  /// encoder readings
  auto radians_per_encoder_difference =
    declare_parameter("radians_per_encoder_difference", 0.00371);

  odom_frame_id = declare_parameter("odom_frame_id", "odom");
  odom_child_frame_id = declare_parameter("odom_child_frame_id", "base_footprint");

  encoder_frequency_lr_to_twist_fl <<  
    meters_per_encoder_sum,
    meters_per_encoder_sum, 
    -radians_per_encoder_difference, +radians_per_encoder_difference;

  auto pi_p = declare_parameter("motor_control_gain_p", 0.0005);
  auto pi_i = declare_parameter("motor_control_gain_i", 0.004);
  auto pi_windup = declare_parameter("motor_control_windup", 100.0);
  auto now = get_clock()->now();

  left_motor_controller = std::make_unique<PIController>(pi_p, pi_i, pi_windup, now);
  right_motor_controller = std::make_unique<PIController>(pi_p, pi_i, pi_windup, now);
}

/// Takes a number between -1.0 and +1.0 and converts it to the nearest motor
/// command value. values out of range will be clamped to the nearest value in
/// range
uint8_t to_motor_command(double d)
{
  if (std::isnan(d)) {
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
  // expecting values in the range of +/- linear_top_speed and +/- angular top
  // speed
  auto linear_rate = msg->linear.x;
  if (top_speed_linear < std::abs(linear_rate)) {
    RCLCPP_WARN(
      get_logger(), "Requested linear velocity %f higher than maximum %f", linear_rate,
      top_speed_linear);
  }

  auto turn_rate = msg->angular.z;
  if (top_speed_angular < std::abs(turn_rate)) {
    RCLCPP_WARN(
      get_logger(), "Requested angular velocity %f higher than maximum %f", turn_rate,
      top_speed_angular);
  }

  Eigen::Vector2d twist_fl(linear_rate, turn_rate);

  RCLCPP_DEBUG(get_logger(), "target velocity = fwd:%f ccw:%f", linear_rate, turn_rate);
  auto encoder_target_freqs = encoder_frequency_lr_to_twist_fl.inverse() * twist_fl;

  auto l_motor = encoder_target_freqs[0];
  auto r_motor = encoder_target_freqs[1];

  left_motor_controller->set_target(l_motor);
  right_motor_controller->set_target(r_motor);

  RCLCPP_DEBUG(get_logger(), "Updated target motor speeds %f %f", l_motor, r_motor);
}

void openrover::Rover::update_odom()
{
  for (auto arg : {data::LeftMotorEncoderState::Which, data::RightMotorEncoderState::Which,
                   data::LeftMotorEncoderPeriod::Which, data::RightMotorEncoderPeriod::Which}) {
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

  if (!left_encoder_position || !right_encoder_position || !left_period || !right_period) {
    RCLCPP_WARN_SKIPFIRST(get_logger(), "Odometry not ready yet");
    return;
  }

  if (odom_last_time.get_clock_type() != RCL_ROS_TIME) {
    RCLCPP_INFO(get_logger(), "Initializing odometry");
    odom_last_encoder_position_left = left_encoder_position->state;
    odom_last_encoder_position_right = right_encoder_position->state;
    odom_last_time = now;

    return;
  }

  if (
    left_encoder_position->time < odom_last_time || right_encoder_position->time < odom_last_time ||
    left_period->time < odom_last_time || right_period->time < odom_last_time) {
    RCLCPP_WARN(get_logger(), "Trying to compute odometry based on stale data");
  }

  // encoder displacement since last time we did odometry.
  auto dt = (now - odom_last_time).seconds();

  // Determine encoder frequency.
  // earlier versions of the firmware don't return the encoder position so we
  // have to make do with the period estimate
  Eigen::Vector2d encoder_frequency_lr;
  if (left_encoder_position->state == 0 && right_encoder_position->state == 0) {
    encoder_frequency_lr = {((left_period->state == 0) ? 0 : 1.0 / (left_period->state)),
                            ((right_period->state == 0) ? 0 : 1.0 / (right_period->state))};
    if (!left_wheel_fwd) {
      encoder_frequency_lr[0] *= -1;
    }
    if (!right_wheel_fwd) {
      encoder_frequency_lr[1] *= -1;
    }
    // ^ the encoder doesn't actually have the wheel direction. So we fake it by
    // assuming the same direction as the last commanded direction we gave to
    // the wheel
  } else {
    encoder_frequency_lr = {
      int16_t(left_encoder_position->state - odom_last_encoder_position_left) / dt,
      int16_t(right_encoder_position->state - odom_last_encoder_position_right) / dt};
    // ^ remember these values are signed. But taking the difference a-b as
    // signed ints will give either a-b or 1<<16 - a-b, whichever has the lower
    // absolute value. This is exactly what we want.
  }

  // drive the motors based on closed-loop control
  {
    auto l_effort = left_motor_controller->step(now, encoder_frequency_lr[0]);
    auto r_effort = right_motor_controller->step(now, encoder_frequency_lr[1]);

    // Assumes direction of effort as the direction of next wheel rotation
    left_wheel_fwd = (l_effort >= 0);
    right_wheel_fwd = (r_effort >= 0);

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
  auto twist_covariance = encoder_frequency_lr_to_twist_fl *
                          encoder_frequency_lr_variance.asDiagonal() *
                          encoder_frequency_lr_to_twist_fl.adjoint();
  {

    //Calculate Odom 
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = odom_frame_id;
    odom->child_frame_id = odom_child_frame_id;
    odom->header.stamp = now;

    // In the odom_frame_id
    odom->pose.covariance.fill(-1.0); //TODO better pose covariance
    odom->pose.pose.position.x = total_lin_x + twist[0] * cos(twist[1]);
    odom->pose.pose.position.y = total_lin_y + twist[0] * sin(twist[1]);
    odom->pose.pose.position.z = 0.0;
    total_lin_x +=  twist[0] * cos(twist[1]);
    total_lin_y += twist[0] * sin(twist[1]);
    total_ang_z += twist[1];
    total_ang_z = fmod(total_ang_z, 6.28); //total_ang_z = total_ang_z % 6.28;
    myQuat.setRPY(0,0,total_ang_z);

    odom->pose.pose.orientation.x = myQuat.x();
    odom->pose.pose.orientation.y = myQuat.y();
    odom->pose.pose.orientation.z = myQuat.z();
    odom->pose.pose.orientation.w = myQuat.w();    
    // In the odom_child_frame_id
    odom->twist.twist.linear.x = twist[0];
    odom->twist.twist.angular.z = twist[1];

    odom->twist.covariance.fill(0.0);
    odom->twist.covariance[0 + 0 * 6] = twist_covariance(0, 0);
    odom->twist.covariance[0 + 5 * 6] = odom->twist.covariance[5 + 0 * 6] = twist_covariance(0, 1);
    odom->twist.covariance[5 + 5 * 6] = twist_covariance(1, 1);

    pub_odom->publish(std::move(odom));

    //update TF;
    if (tf_broadcast == true){
      std::ofstream outfile; //Debug
      outfile.open("test.txt", std::ios_base::app); 
      outfile << total_lin_x << "," << total_lin_y; //End debug
      tf.transform.translation.x = odom->pose.pose.position.x;
      tf.transform.translation.y = odom->pose.pose.position.y;
      tf.transform.translation.z = odom->pose.pose.position.z;
      tf.transform.rotation = odom->pose.pose.orientation; //check
      RCLCPP_DEBUG(get_logger(), "got transforms");
      tf.header.stamp = now;
      tf.header.frame_id = odom_frame_id;
      tf.child_frame_id = odom_child_frame_id;
      br->sendTransform(tf);
      RCLCPP_DEBUG(get_logger(), "sent");
    }
    

  }

  odom_last_encoder_position_left = left_encoder_position->state;
  odom_last_encoder_position_right = right_encoder_position->state;
  odom_last_time = now;
}

void openrover::Rover::on_raw_data(openrover_core_msgs::msg::RawData::ConstSharedPtr data)
{
  most_recent_data[data->which] =
    std::make_unique<Timestamped<data::RawValue>>(get_clock()->now(), data->value);
}

void Rover::update_power_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  // request the data for next time we call
  for (auto which : {data::BatteryChargingState::which(), data::BatteryAStateOfCharge::which(),
                     data::BatteryBStateOfCharge::which(), data::BatteryACurrent::which(),
                     data::BatteryBCurrent::which(), data::BatteryACurrentInternal::which(),
                     data::BatteryBCurrentInternal::which()}) {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = which;
    pub_rover_command->publish(cmd);
  }

  status.clearSummary();

  if (auto data = get_recent<data::BatteryAStateOfCharge>()) {
    status.add("battery A state of charge", data->state);
    if (data->state < 0.1) {
      status.mergeSummary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN, "Battery A is low on charge");
    }
  }
  if (auto data = get_recent<data::BatteryBStateOfCharge>()) {
    status.add("battery B state of charge", data->state);
    if (data->state < 0.1) {
      status.mergeSummary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN, "Battery B is low on charge");
    }
  }
  if (auto data = get_recent<data::BatteryACurrent>()) {
    status.add("battery A current", data->state);
  }
  if (auto data = get_recent<data::BatteryBCurrent>()) {
    status.add("battery B current", data->state);
  }
  if (auto data = get_recent<data::BatteryACurrentInternal>()) {
    status.add("battery A current (internal)", data->state);
    if (data->state < -9.9) {
      status.mergeSummary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN, "Battery A is drawing high current");
    }
  }
  if (auto data = get_recent<data::BatteryBCurrentInternal>()) {
    status.add("battery B current (internal)", data->state);
    if (data->state < -9.9) {
      status.mergeSummary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN, "Battery B is drawing high current");
    }
  }
  if (auto data = get_recent<data::BatteryChargingState>()) {
    status.add("battery charging state", data->state);
  }

  if (status.message.empty()) status.message = "Okay";
}

void Rover::update_firmware_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  openrover_core_msgs::msg::RawCommand cmd;
  cmd.verb = 10;
  cmd.arg = data::RoverVersion::which();
  pub_rover_command->publish(cmd);

  status.clearSummary();

  if (auto data = get_recent<data::RoverVersion>()) {
    status.add("firmware revision", data->state.to_string());
  } else {
    status.mergeSummary(
      diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Could not get firmware version");
  }

  if (status.message.empty()) status.message = "Okay";
}
void Rover::update_drive_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  // Ask the rover to send the next batch of diagnostics data
  for (auto which : {data::MotorTemperature1::which(), data::LeftMotorStatus::which(),
                     data::RightMotorStatus::which(), data::FlipperMotorStatus::which(),
                     data::CoolingFan1DutyFactor::which(), data::CoolingFan2DutyFactor::which()}) {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = which;
    pub_rover_command->publish(cmd);
  }

  status.clearSummary();

  if (auto data = get_recent<data::LeftMotorEncoderState>()) {
    status.add("left encoder displacement", data->state);
  }
  if (auto data = get_recent<data::RightMotorEncoderState>()) {
    status.add("right encoder displacement", data->state);
  }
  if (auto data = get_recent<data::LeftMotorEncoderPeriod>()) {
    status.add("left encoder period", data->state);
  }
  if (auto data = get_recent<data::RightMotorEncoderPeriod>()) {
    status.add("right encoder period", data->state);
  }
  if (auto data = get_recent<data::MotorTemperature1>()) {
    status.add("left motor temperature", data->state);
    if (data->state > 50.0)
      status.mergeSummary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN, "left motor is too hot");
  }
  if (auto data = get_recent<data::MotorTemperature2>()) {
    status.add("right motor temperature", data->state);
    if (data->state > 50.0)
      status.mergeSummary(
        diagnostic_updater::DiagnosticStatusWrapper::WARN, "right motor is too hot");
  }
  if (auto data = get_recent<data::CoolingFan1DutyFactor>()) {
    status.add("cooling fan 1 duty factor", data->state);
  }
  if (auto data = get_recent<data::CoolingFan2DutyFactor>()) {
    status.add("cooling fan 2 duty factor", data->state);
  }

  if (status.message.empty()) status.message = "Okay";
}