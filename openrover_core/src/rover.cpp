#include "rover.hpp"
#include <algorithm>
#include <chrono>

#include <math.h>
#include "data.hpp"

using std::placeholders::_1;
using namespace openrover;
using namespace std::literals::chrono_literals;
using geometry_msgs::msg::Vector3;
using namespace openrover_core_msgs;

using Cls = Rover;

Rover::Rover() : Node("openrover", "", true)
{
  sub_raw_data = create_subscription<msg::RawData>("raw_data", std::bind(&Cls::on_raw_data, this, _1), 20);
  tmr_diagnostics = create_wall_timer(1000ms, std::bind(&Cls::update_diagnostics, this));
  tmr_odometry = create_wall_timer(30ms, std::bind(&Cls::update_odometry, this));

  sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", std::bind(&Cls::on_velocity, this, _1), 1);
  pub_rover_command = create_publisher<openrover_core_msgs::msg::RawCommand>("openrover_command", 20);
  pub_motor_efforts = create_publisher<openrover_core_msgs::msg::RawMotorCommand>("motor_efforts", 1);
  pub_diagnostics = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  pub_obs_vel = create_publisher<geometry_msgs::msg::Twist>("obs_vel");

  // based on the physical capabilities of the rover. Depends on the wheel configuration (2wd/4wd/treads)
  if (!get_parameter<double>("top_speed_linear", top_speed_linear) || !(top_speed_linear > 0))
  {
    top_speed_linear = 1.5;
    RCLCPP_WARN(get_logger(), "Top linear speed not known. setting to %f", top_speed_linear);
  }

  // based on the physical capabilities of the rover. Depends on the wheel configuration (2wd/4wd/treads)
  if (!get_parameter<double>("top_speed_angular", top_speed_angular) || !(top_speed_angular > 0))
  {
    top_speed_angular = 6.2;
    RCLCPP_WARN(get_logger(), "Top turning speed not known. setting to %f", top_speed_angular);
  }

  if (!get_parameter<double>("meters_per_encoder_unit", meters_per_encoder_unit) || !(meters_per_encoder_unit > 0))
  {
    meters_per_encoder_unit = 0.005;  // todo: calibrate this value
    RCLCPP_WARN(get_logger(), "Odometry linear scale not known. setting to %f", meters_per_encoder_unit);
  }
  if (!get_parameter<double>("radians_per_delta_encoder_unit", radians_per_delta_encoder_unit) ||
      !(radians_per_delta_encoder_unit > 0))
  {
    radians_per_delta_encoder_unit = 0.02;  // todo: calibrate this value
    RCLCPP_WARN(get_logger(), "Odometry turning scale not known. setting to %f", radians_per_delta_encoder_unit);
  }
}

/// Takes a number between -1.0 and +1.0 and converts it to the nearest motor speed.
/// values out of range will be converted to the nearest value in range
uint8_t to_motor_speed(double d)
{
  if (isnan(d))
  {
    return 125;
  }
  d = trunc(d * 125);
  d = std::min(d, +125.0);
  d = std::max(d, -125.0);
  d += 125;
  return static_cast<uint8_t>(d);
}

void Rover::on_velocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
  // expecting values in the range of +/- linear_top_speed and +/- angular top speed
  auto linear_rate = msg->linear.x;
  if (!(std::abs(linear_rate) < top_speed_linear))
  {
    RCLCPP_WARN(get_logger(), "Requested linear velocity %f higher than maximum %f", linear_rate, top_speed_linear);
  }

  auto turn_rate = msg->angular.z;
  if (!(std::abs(turn_rate) < top_speed_angular))
  {
    RCLCPP_WARN(get_logger(), "Requested angular velocity %f higher than maximum %f", turn_rate, top_speed_angular);
  }

  // convert the requested speeds to per-motor speeds of [-1.0,+1.0]
  double l_motor = (linear_rate / top_speed_linear) + (turn_rate / top_speed_angular / 2.0);
  double r_motor = (linear_rate / top_speed_linear) - (turn_rate / top_speed_angular / 2.0);

  // and translate them to the hardware values [0, 250]
  openrover_core_msgs::msg::RawMotorCommand e;
  e.left = to_motor_speed(l_motor);
  e.right = to_motor_speed(r_motor);
  e.flipper = to_motor_speed(0);  // todo

  pub_motor_efforts->publish(e);
}

void openrover::Rover::update_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostic_msgs::msg::DiagnosticStatus encoders_status;
  encoders_status.hardware_id = "openrover/motor encoders";

  if (auto left = get_recent<data::LeftMotorEncoderState>())
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "left";
    kv.value = left->state.string_value();
    encoders_status.values.push_back(kv);
  }
  if (auto right = get_recent<data::RightMotorEncoderState>())
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "right";
    kv.value = right->state.string_value();
    encoders_status.values.push_back(kv);
  }
  if (auto flipper = get_recent<data::FlipperMotorEncoderState>())
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "flipper";
    kv.value = flipper->state.string_value();
    encoders_status.values.push_back(kv);
  }
  diagnostics.status.push_back(encoders_status);
  pub_diagnostics->publish(diagnostics);
}

void openrover::Rover::update_odometry()
{
  for (auto arg : { data::LeftMotorEncoderState::Which, data::RightMotorEncoderState::Which })
  {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = arg;
    pub_rover_command->publish(cmd);
  }

  auto left_encoder_recent = get_recent<data::LeftMotorEncoderState>();
  auto right_encoder_recent = get_recent<data::RightMotorEncoderState>();

  if (!left_encoder_recent || !right_encoder_recent)
  {
    RCLCPP_WARN(get_logger(), "No odometry data yet");
    return;
  }

  if (left_encoder_published && right_encoder_published)
  {
    auto left_encoder_delta_state = left_encoder_recent->state.get_value() - left_encoder_published->state.get_value();
    // ^ remember these values are signed. But taking the difference a-b as signed ints will give either a-b or 1<<16 -
    // a-b, whichever has the lower absolute value. This is exactly what we want.
    auto left_encoder_deltatime = (left_encoder_recent->time - left_encoder_published->time).seconds();

    auto right_encoder_delta_state =
        right_encoder_recent->state.get_value() - right_encoder_published->state.get_value();
    auto right_encoder_deltatime = (right_encoder_recent->time - right_encoder_published->time).seconds();

    if (left_encoder_deltatime == 0.0 || right_encoder_deltatime == 0.0)
    {
      RCLCPP_WARN(get_logger(), "Could not publish observed velocity due to lack of new data");
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x =
        (left_encoder_delta_state / left_encoder_deltatime + right_encoder_delta_state / right_encoder_deltatime) *
        meters_per_encoder_unit;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z =
        (left_encoder_delta_state / left_encoder_deltatime - right_encoder_delta_state / right_encoder_deltatime) *
        meters_per_encoder_unit;

    pub_obs_vel->publish(twist);
  };

  left_encoder_published = left_encoder_recent;
  right_encoder_published = right_encoder_recent;
}

void openrover::Rover::on_raw_data(openrover_core_msgs::msg::RawData::SharedPtr data)
{
  Timestamped<std::array<uint8_t, 2>> ts_data = { get_clock()->now(), data->value };
  most_recent_data.emplace(data->which, ts_data);
}
