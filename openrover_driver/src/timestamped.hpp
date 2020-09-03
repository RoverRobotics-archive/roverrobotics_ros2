#pragma once
#include "rclcpp/rclcpp.hpp"
namespace openrover
{
template <typename T>
struct Timestamped
{
  Timestamped(const rclcpp::Time & time, T state) : time(time), state(state) {}

  using State = T;
  rclcpp::Time time;
  T state;
};

}  // namespace openrover
