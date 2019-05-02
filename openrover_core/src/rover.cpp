#include "rover.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

#include <math.h>
#include "data.hpp"

using std::placeholders::_1;
using namespace openrover;
using namespace std::literals::chrono_literals;
using geometry_msgs::msg::Vector3;
using namespace openrover_core_msgs;

using Cls = Rover;

bool is_positive(double x) { return x > 0.0; }

Rover::Rover() : Node("openrover", "", true)
{
  sub_raw_data = create_subscription<msg::RawData>("raw_data", std::bind(&Cls::on_raw_data, this, _1), 20);
  tmr_diagnostics = create_wall_timer(1000ms, std::bind(&Cls::update_diagnostics, this));
  tmr_odometry = create_wall_timer(100ms, std::bind(&Cls::update_motor_distances, this));

  sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", std::bind(&Cls::on_velocity, this, _1), 1);
  pub_rover_command = create_publisher<openrover_core_msgs::msg::RawCommand>("openrover_command", 20);
  pub_motor_efforts = create_publisher<openrover_core_msgs::msg::RawMotorCommand>("motor_efforts", 1);
  pub_diagnostics = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  pub_obs_vel = create_publisher<geometry_msgs::msg::Twist>("obs_vel");

  // based on the physical capabilities of the rover. Depends on the wheel configuration (2wd/4wd/treads) and terrain
  top_speed_linear = get_parameter_checked<double>("top_speed_linear", &is_positive, 6.40);
  top_speed_angular = get_parameter_checked<double>("top_speed_angular", &is_positive, 6.45);
  // this value determined by driving straight and dividing distance by average encoder reading
  meters_per_encoder_unit = get_parameter_checked<double>("meters_per_encoder_unit", &is_positive, 0.001375);
  // this value determined by driving in a circle and dividing rotation by difference in encoder readings
  radians_per_delta_encoder_unit =
      get_parameter_checked<double>("radians_per_delta_encoder_unit", &is_positive, 0.00522);
}

/// Takes a number between -1.0 and +1.0 and converts it to the nearest motor speed.
/// values out of range will be converted to the nearest value in range
uint8_t to_motor_speed(double d)
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
  diagnostics.header.stamp = get_clock()->now();
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
    diagnostics.status.push_back(rover_status);
  }
  {
    diagnostic_msgs::msg::DiagnosticStatus encoders_status;
    encoders_status.hardware_id = "openrover/motor encoders";
    if (auto left = get_recent<data::LeftMotorEncoderState>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "left";
      kv.value = std::to_string(left->state);
      encoders_status.values.push_back(kv);
    }
    if (auto right = get_recent<data::RightMotorEncoderState>())
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "right";
      kv.value = std::to_string(right->state);
      encoders_status.values.push_back(kv);
    }
    diagnostics.status.push_back(encoders_status);
  }
  pub_diagnostics->publish(diagnostics);
}

void openrover::Rover::update_motor_distances()
{
  for (auto arg : { data::LeftMotorEncoderState::which(), data::RightMotorEncoderState::which() })
  {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = arg;
    pub_rover_command->publish(cmd);
  }

  auto left_new_state = get_recent<data::LeftMotorEncoderState>();
  auto right_new_state = get_recent<data::RightMotorEncoderState>();

  if (!left_new_state || !right_new_state)
  {
    RCLCPP_WARN(get_logger(), "No odometry data yet");
    return;
  }

  if (!encoder_left_published_state || !encoder_right_published_state)
  {
    // prime with data
    RCLCPP_INFO(get_logger(), "Initializing encoders");

    encoder_left_published_state = std::move(left_new_state);
    encoder_right_published_state = std::move(right_new_state);
    return;
  }

  auto left_delta_state = left_new_state->state - encoder_left_published_state->state;
  // ^ remember these values are signed. But taking the difference a-b as signed ints will give either a-b or 1<<16 -
  // a-b, whichever has the lower absolute value. This is exactly what we want.
  auto left_delta_time = (left_new_state->time() - encoder_left_published_state->time()).seconds();

  auto right_delta_state = right_new_state->state - encoder_right_published_state->state;
  auto right_delta_time = (right_new_state->time() - encoder_right_published_state->time()).seconds();

  if (left_delta_time == 0.0 || right_delta_time == 0.0)
  {
    RCLCPP_WARN(get_logger(), "Could not publish observed velocity due to lack of new data");
    RCLCPP_WARN(get_logger(), "old: %d %f new: %d %f", encoder_left_published_state->state,
                encoder_left_published_state->time().seconds(), left_new_state->state,
                left_new_state->time().seconds());
    return;
  }
  else
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x =
        (left_delta_state / left_delta_time + right_delta_state / right_delta_time) * meters_per_encoder_unit;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z =
        (left_delta_state / left_delta_time - right_delta_state / right_delta_time) * meters_per_encoder_unit;

    pub_obs_vel->publish(twist);

    encoder_left_published_state = std::move(left_new_state);
    encoder_right_published_state = std::move(right_new_state);
  }
}

void openrover::Rover::on_raw_data(openrover_core_msgs::msg::RawData::SharedPtr data)
{
  most_recent_data[data->which] = std::make_unique<Timestamped<data::RawValue>>(get_clock()->now(), data->value);
}
