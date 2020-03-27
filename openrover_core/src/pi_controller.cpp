//
// Created by snuc on 6/19/19.
//

#include "pi_controller.hpp"

// small value for numerical stability when accelerating from near-zero
const double EPSILON = 0.001;

double clamp(double value, double min, double max)
{
  assert(min <= max);
  return (value < min) ? min : (max < value) ? max : value;
}

PIController::PIController(
    double proportional_gain, double integral_gain, double windup_limit,
    const rclcpp::Time & time_zero)
    : proportional_gain(proportional_gain),
      integral_gain(integral_gain),
      windup_limit(windup_limit),
      last_time(time_zero),
      target(0.0),
      error_integral(0.0),
      control_value(0.0)
{
  assert(proportional_gain >= 0);
  assert(integral_gain >= 0);
  assert(windup_limit > 0);
  assert(proportional_gain || integral_gain);
}

double PIController::step(const rclcpp::Time & now, double measured_value)
{
  assert(last_time < now);
  auto error = target - measured_value;
  error_integral += error * (now - last_time).seconds();
  error_integral = clamp(error_integral, -windup_limit, +windup_limit);
  last_time = now;
  control_value = error * proportional_gain + error_integral * integral_gain;

  return control_value;
}

void PIController::set_target(double new_target) {
  // Assuming near-proportionality, this stabilizes the filter with a changing target.
  // This term means if we negate the direction, we negate the integrated error.
  // and if we halve the speed, we halve the integrated error.
  if (abs(target) <= EPSILON) {
    error_integral *= new_target / target;
  }
  target = new_target;
}
