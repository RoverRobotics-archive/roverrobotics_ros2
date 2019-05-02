#pragma once
#include "rclcpp/rclcpp.hpp"
namespace openrover
{
template <typename T>
struct Timestamped
{
  Timestamped(rcl_time_point_value_t nanoseconds, T state) : nanoseconds(nanoseconds), state(state) {}
  Timestamped(const rclcpp::Time& time, T state) : nanoseconds(time.nanoseconds()), state(state) {}

  using State = T;
  rcl_time_point_value_t nanoseconds;
  rclcpp::Time time() const { return rclcpp::Time(nanoseconds); }
  T state;
};

}  // namespace openrover