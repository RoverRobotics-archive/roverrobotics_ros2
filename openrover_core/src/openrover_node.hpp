#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "atomic"
#include <chrono>
#include "openrover_core_msgs/msg/rover_data_raw.hpp"
#include "openrover_core_msgs/msg/motor_efforts.hpp"
#include "openrover_core_msgs/msg/rover_command_raw.hpp"
using namespace openrover_core_msgs;
using namespace std::chrono_literals;


class OpenRoverNode : public rclcpp::Node {
public:
	OpenRoverNode();

protected:
	std::atomic<std::array<uint8_t, 3>> motor_efforts_u8;

	/// Should send messages at this frequency, even if no data is requested.
	std::chrono::milliseconds keepalive_period = 100ms;

	/// If no motor speed is commanded for this long, kill the motors
	std::chrono::milliseconds kill_motors_timeout = 333ms;

	std::chrono::microseconds uart_poll_period= 100us;

	rclcpp::Publisher<openrover_core_msgs::msg::RoverDataRaw>::SharedPtr pub_raw_data;
	rclcpp::Subscription<openrover_core_msgs::msg::MotorEfforts>::SharedPtr sub_motor_efforts;
	rclcpp::Subscription<openrover_core_msgs::msg::RoverCommandRaw>::SharedPtr sub_raw_commands;

	void on_raw_command(openrover_core_msgs::msg::RoverCommandRaw::SharedPtr);
	void read_callback();
	void keepalive_callback();
	void on_kill_motors();
	void on_motor_speed_commanded(openrover_core_msgs::msg::MotorEfforts::SharedPtr msg);
	rclcpp::TimerBase::SharedPtr keepalive_timer;
	rclcpp::TimerBase::SharedPtr read_timer;
	rclcpp::TimerBase::SharedPtr kill_motors_timer;

	std::unique_ptr<serial::Serial> serial;
};

class OpenRoverError : public std::runtime_error {
public:
	OpenRoverError(const char* msg) : std::runtime_error(msg) {};
	OpenRoverError(const std::string msg) : std::runtime_error(msg) {};
};