#include "openrover_node.hpp"
#include <algorithm>
using std::placeholders::_1;
using Cls = OpenRoverNode;

OpenRoverNode::OpenRoverNode(): Node("openrover","",true)
{
	this->sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel",std::bind(&Cls::on_velocity, this, _1), 1);
	this->pub_motor_efforts = create_publisher<openrover_core_msgs::msg::RawMotorCommand>("motor_efforts", 1);
}

/// Takes a number between -125.0 and +125.0 and converts it to the nearest motor speed.
/// values out of range will be converted to the nearest value in range
uint8_t to_motor_speed(double d) {
	if (isnan(d)) {
		d = 0.0;
	}
	d = trunc(d);
	d = std::min(d, +125.0);
	d = std::max(d, -125.0);
	d += 125;
	return static_cast<uint8_t>(d);
}

void OpenRoverNode::on_velocity(geometry_msgs::msg::Twist::SharedPtr msg)
{
	const double ODOM_AXLE_TRACK_2WD = 14.375; //distance between wheels
	const double ODOM_ANGULAR_COEF_2WD = 1.0 / (ODOM_AXLE_TRACK_2WD * 2.54 / 100); //rad per meter

	const int MOTOR_SPEED_LINEAR_COEF_2WD = 293;
	const int MOTOR_SPEED_ANGULAR_COEF_2WD = 56;

	auto linear_rate = msg->linear.x;
	auto turn_rate = msg->angular.z;
	auto diff_motors = turn_rate / ODOM_ANGULAR_COEF_2WD;

	double r_motor = (linear_rate * MOTOR_SPEED_ANGULAR_COEF_2WD) + (turn_rate * MOTOR_SPEED_ANGULAR_COEF_2WD);
	double l_motor = trunc((linear_rate * MOTOR_SPEED_ANGULAR_COEF_2WD) - (turn_rate * MOTOR_SPEED_ANGULAR_COEF_2WD));
	
	openrover_core_msgs::msg::RawMotorCommand e;
	e.left = to_motor_speed(l_motor);
	e.right = to_motor_speed(r_motor);
	e.flipper = to_motor_speed(0); // todo

	pub_motor_efforts->publish(e);
}
