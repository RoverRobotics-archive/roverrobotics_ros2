#include "rover.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include "tf2/convert.h"
#include <math.h>
#include "data.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;
using namespace openrover;
using namespace std::literals::chrono_literals;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using namespace openrover_core_msgs;

using Cls = Rover;

bool is_positive(double x) { return x > 0.0; }

Rover::Rover() : Node("openrover", "", true)
{
  sub_raw_data = create_subscription<msg::RawData>("raw_data", std::bind(&Cls::on_raw_data, this, _1), 20);
  tmr_diagnostics = create_wall_timer(1000ms, std::bind(&Cls::update_diagnostics, this));
  tmr_odometry = create_wall_timer(100ms, std::bind(&Cls::update_odom, this));

  sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", std::bind(&Cls::on_cmd_vel, this, _1), 1);
  pub_rover_command = create_publisher<openrover_core_msgs::msg::RawCommand>("openrover_command", 20);
  pub_motor_efforts = create_publisher<openrover_core_msgs::msg::RawMotorCommand>("motor_efforts", 1);
  pub_diagnostics = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  pub_obs_vel = create_publisher<geometry_msgs::msg::Twist>("obs_vel");
  pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom");

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

void Rover::on_cmd_vel(geometry_msgs::msg::Twist::SharedPtr msg)
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

  // convert the requested speeds to per-motor speeds of [-1.0,+1.0]
  double l_motor = (linear_rate / top_speed_linear) + (turn_rate / top_speed_angular / 2.0);
  left_wheel_fwd = l_motor >= 0;
  double r_motor = (linear_rate / top_speed_linear) - (turn_rate / top_speed_angular / 2.0);
  right_wheel_fwd = r_motor >= 0;

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
    if (auto data = get_recent<data::LeftMotorEncoderInterval>())
    {
    }
    diagnostics.status.push_back(encoders_status);
  }
  pub_diagnostics->publish(diagnostics);
}

void openrover::Rover::update_odom()
{
  for (auto arg : { data::LeftMotorEncoderState::Which, data::RightMotorEncoderState::Which,
                    data::LeftMotorEncoderInterval::Which, data::RightMotorEncoderInterval::Which })
  {
    openrover_core_msgs::msg::RawCommand cmd;
    cmd.verb = 10;
    cmd.arg = arg;
    pub_rover_command->publish(cmd);
  }
  if (odom_last_updated.nanoseconds() == 0)
  {
    odom_last_updated = get_clock()->now();
    odom_last_pos_x = 0;
    odom_last_pos_y = 0;
    odom_last_yaw = 0;
    return;
  }
  auto dt = (get_clock()->now() - odom_last_updated).seconds();
  odom_last_updated = get_clock()->now();

  auto left_encoder_position = get_recent<data::LeftMotorEncoderState>();
  auto right_encoder_position = get_recent<data::RightMotorEncoderState>();
  auto left_period = get_recent<data::LeftMotorEncoderInterval>();
  auto right_period = get_recent<data::RightMotorEncoderInterval>();

  if (left_encoder_position->time() < odom_last_updated || right_encoder_position->time() < odom_last_updated ||
      left_period->time() < odom_last_updated || right_period->time() < odom_last_updated)
  {
    RCLCPP_WARN(get_logger(), "Odometry based on stale data");
  }

  double left_encoder_frequency = left_period->state == 0 ? 0 : 1.0 / left_period->state;
  left_encoder_frequency *= left_wheel_fwd ? +1 : -1;
  // ^ the encoder doesn't actually have the wheel direction. So we fake it by assuming the same direction as the last
  // commanded direction we gave to the wheel
  double right_encoder_frequency = right_period->state == 0 ? 0 : 1.0 / right_period->state;
  right_encoder_frequency *= right_wheel_fwd ? +1 : -1;

  double left_encoder_displacement, right_encoder_displacement;

  // earlier versions of the firmware don't return the encoder position so we have to do it all based on the encoder
  // frequency
  if (left_encoder_position->state == 0 && right_encoder_position->state == 0)
  {
    left_encoder_displacement = left_encoder_frequency * dt;
    right_encoder_displacement = right_encoder_frequency * dt;
  }
  else
  {
    left_encoder_displacement = left_encoder_position->state - encoder_last_position_left;
    // ^ remember these values are signed. But taking the difference a-b as signed ints will give either a-b or 1<<16 -
    // a-b, whichever has the lower absolute value. This is exactly what we want.
    right_encoder_displacement = right_encoder_position->state - encoder_last_position_right;
  }

  auto angle = odom_last_yaw;
  auto displacement_linear = (left_encoder_displacement + right_encoder_displacement) / 2 * meters_per_encoder_unit;
  auto displacement_angular = (left_encoder_displacement - right_encoder_displacement) * radians_per_delta_encoder_unit;

  // forward velocity relative to the robot's current frame of reference
  auto velocity_forward = (left_encoder_frequency + right_encoder_frequency) / 2 * meters_per_encoder_unit;
  // rotational velocity relative to the robot's current frame of reference
  auto velocity_clockwise = (left_encoder_frequency - right_encoder_frequency) * radians_per_delta_encoder_unit;

  odom_last_pos_x += cos(angle + displacement_angular / 2) * displacement_linear;
  odom_last_pos_y += sin(angle + displacement_angular / 2) * displacement_linear;
  odom_last_yaw += displacement_angular;

  {
    Twist obs_vel;
    obs_vel.linear.x = velocity_forward;
    obs_vel.angular.z = velocity_clockwise;
    pub_obs_vel->publish(obs_vel);
  }

  {
    nav_msgs::msg::Odometry odom;
    tf2::Quaternion pose_q;
    pose_q.setRPY(0, 0, odom_last_yaw);
    odom.pose.pose.position.x = odom_last_pos_x;
    odom.pose.pose.position.y = odom_last_pos_y;
    odom.pose.pose.orientation = toMsg(pose_q);

    odom.twist.twist.linear.x = velocity_forward * cos(odom_last_yaw);
    odom.twist.twist.linear.y = velocity_forward * sin(odom_last_yaw);
    odom.twist.twist.angular.z = velocity_clockwise;
    pub_odom->publish(odom);
  }
}

void openrover::Rover::on_raw_data(openrover_core_msgs::msg::RawData::SharedPtr data)
{
  most_recent_data[data->which] = std::make_unique<Timestamped<data::RawValue>>(get_clock()->now(), data->value);
}
