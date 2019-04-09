#include <chrono>
#include "std_msgs/msg/string.hpp"
#include "openrover_node.hpp"

using std::placeholders::_1;
using namespace openrover_core_msgs;
const uint8_t UART_START_PACKET = 253;
unsigned long BAUDRATE = 57600;

const std::array<uint8_t, 3> MOTOR_EFFORT_HALT = { 125,125,125 };

uint8_t checksum(std::vector<uint8_t> payload) {
	uint32_t sum = 0;
	for (auto i : payload) {
		sum += i;
	}
	return 255 - (sum % 255);
}

std::vector<uint8_t> packetize(std::vector<uint8_t> payload) {
	std::vector<uint8_t> result;
	result.push_back(UART_START_PACKET);
	result.insert(result.end(), payload.begin(), payload.end());
	result.push_back(checksum(payload));
	return result;
}

std::vector<uint8_t> depacketize(std::vector<uint8_t> packet) {
	if (packet[0] != UART_START_PACKET)
		throw new OpenRoverError("Bad packet: did not start with a start byte");

	std::vector<uint8_t> payload;
	for (auto i = 1; i < packet.size() - 1; i++) {
		payload.push_back(packet[i]);
}

	auto expected_checksum = checksum(payload);
	auto received_checksum = packet[packet.size() - 1];
	if (expected_checksum != received_checksum)
		throw new OpenRoverError("Bad packet: incorrect checksum");

	return payload;
}

OpenRoverNode::OpenRoverNode() : Node("openrover_core"), motor_efforts_u8(MOTOR_EFFORT_HALT) {
	keepalive_timer = this->create_wall_timer(keepalive_period, std::bind(&OpenRoverNode::keepalive_callback, this));

	 reader_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
	
	read_timer = this->create_wall_timer(uart_poll_period, std::bind(&OpenRoverNode::read_callback, this))//, reader_group); 

	auto writer_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
	kill_motors_timer = this->create_wall_timer(kill_motors_timeout, std::bind(&OpenRoverNode::on_kill_motors, this));

	pub_raw_data = this->create_publisher<openrover_core_msgs::msg::RoverDataRaw>("raw_data");

	sub_motor_efforts = this->create_subscription<msg::MotorEfforts>("motor_speeds_commanded", std::bind(&OpenRoverNode::on_motor_speed_commanded, this, _1), 1); //, writer_group);
	sub_raw_commands = this->create_subscription<msg::RoverCommandRaw>("raw_command", std::bind(&OpenRoverNode::on_raw_command, this, _1), 16); // , writer_group);

	std::string port = "COM3";
	RCLCPP_INFO(this->get_logger(), "Connecting to serial: '%s'", port.c_str());
	serial = std::make_unique<serial::Serial>(port, BAUDRATE, serial::Timeout::simpleTimeout(1000));
	if (!
		serial->isOpen()) {

		RCLCPP_ERROR(this->get_logger(), "could not open serial device");
		throw OpenRoverError("could not open serial device");
	}
}

void OpenRoverNode::on_raw_command(openrover_core_msgs::msg::RoverCommandRaw::SharedPtr cmd)
{
	auto efforts = motor_efforts_u8.load();
	std::vector<uint8_t> payload{ efforts[0],efforts[1],efforts[2], cmd->verb, cmd->arg };
	auto packed = packetize(payload);
	this->serial->write(packed);
	keepalive_timer->reset();
}

void OpenRoverNode::keepalive_callback() {
	msg::RoverCommandRaw cmd;
	cmd.verb = 10;
	cmd.arg = 40;
	auto efforts = motor_efforts_u8.load();
	std::vector<uint8_t> payload{efforts[0], efforts[1], efforts[2], cmd.verb, cmd.arg };
	auto packed = packetize(payload);
	this->serial->write(packed);
}

std::string strhex(std::vector<uint8_t> bin) {
	std::ostringstream ss;
	for (auto h : bin) {
		char buf[10];
		sprintf(buf, "%0x", h);
			ss << buf;
	}
	
	return ss.str();
}

void OpenRoverNode::read_callback()
{
	const size_t READ_PACKET_SIZE = 5;
	std::vector<uint8_t> inbuf;

	while (inbuf.size() == 0) {
		if (serial->available() < READ_PACKET_SIZE)
			return; // not enough data to possibly form a full packet

		auto n = serial->read(inbuf, 1);
		if (n == 0)
			return; // timed out

		if (inbuf[0] != UART_START_PACKET) {
			RCLCPP_WARN(this->get_logger(), "Expected start byte, instead got: %0x", inbuf[0]);
			inbuf.clear();
		}
	}

	auto t = serial->read(inbuf, READ_PACKET_SIZE-1);

	RCLCPP_DEBUG(this->get_logger(), "Packet: %s.", strhex(inbuf));

	auto payload = depacketize(inbuf);

	msg::RoverDataRaw data;
	data.data_element = payload[0];
	data.data_value = { payload[1] , payload[2] };

	this->pub_raw_data->publish(data);
}

void OpenRoverNode::on_kill_motors()
{
	motor_efforts_u8 = MOTOR_EFFORT_HALT;
}


uint8_t r_to_u8(float e) {
	return (uint8_t)std::trunc(e);
}

void OpenRoverNode::on_motor_speed_commanded(openrover_core_msgs::msg::MotorEfforts::SharedPtr msg)
{
	std::array<float, 3> new_efforts_f = { msg->left, msg->right, msg->flipper };
	std::array<uint8_t, 3> new_efforts_u8;
	bool okay = true;
	for (auto i = 0; i < 3; i++) {
		if (std::isnan(new_efforts_f[i]) || new_efforts_f[i] > 1.0f || new_efforts_f[i] < -1.0f) {
			okay = false;
			break;
		}
		new_efforts_u8[i] = (uint8_t)std::truncf(new_efforts_f[i] * 125 + 125);
	}

	if (okay) {
		this->motor_efforts_u8 = new_efforts_u8;
		this->kill_motors_timer->reset();
	}
	else {
		RCLCPP_WARN(this->get_logger(), "Received invalid motor speed command: %s",msg);
	}
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OpenRoverNode>());
	rclcpp::shutdown();
	return 0;
}