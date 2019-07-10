#include "rover_serial.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

using serial::Serial;
using namespace openrover_core_msgs;
using namespace openrover;

const std::array<uint8_t, 3> MOTOR_EFFORT_HALT = {125, 125, 125};

uint8_t checksum(const std::vector<uint8_t> & payload)
{
  uint32_t sum = 0;
  for (auto i : payload) {
    sum += i;
  }
  return 255 - (sum % 255);
}

std::vector<uint8_t> packetize(std::vector<uint8_t> payload)
{
  std::vector<uint8_t> result;
  result.push_back(UART_START_PACKET);
  result.insert(result.end(), payload.begin(), payload.end());
  result.push_back(checksum(payload));
  return result;
}

std::vector<uint8_t> depacketize(std::vector<uint8_t> packet)
{
  if (packet[0] != UART_START_PACKET) {
    throw OpenRoverError("Bad packet: did not start with a start byte");
  }

  std::vector<uint8_t> payload;
  for (size_t i = 1; i < packet.size() - 1; i++) {
    payload.push_back(packet[i]);
  }

  auto expected_checksum = checksum(payload);
  auto received_checksum = packet[packet.size() - 1];
  if (expected_checksum != received_checksum) {
    throw OpenRoverError("Bad packet: incorrect checksum");
  }

  return payload;
}

RoverSerial::RoverSerial()
: Node("rover_serial", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Starting rover serial communication node");

  std::string serial_port = declare_parameter("serial_port", "/dev/ttyUSB0");

  motor_efforts_u8 = std::make_shared<std::array<uint8_t, 3>>(MOTOR_EFFORT_HALT);

  keepalive_timer = create_wall_timer(keepalive_period, [=]() { keepalive_callback(); });
  read_timer = create_wall_timer(uart_poll_period, [=]() { read_callback(); });

  // auto writer_group =
  // this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  kill_motors_timer = create_wall_timer(kill_motors_timeout, [=]() { on_kill_motors(); });

  pub_raw_data = create_publisher<openrover_core_msgs::msg::RawData>("raw_data", rclcpp::QoS(16));

  sub_motor_efforts = create_subscription<msg::RawMotorCommand>(
    "motor_efforts", rclcpp::QoS(1),
    [=](msg::RawMotorCommand::SharedPtr msg) { on_motor_efforts(msg); });
  sub_raw_commands = create_subscription<msg::RawCommand>(
    "openrover_command", rclcpp::QoS(16),
    [=](msg::RawCommand::SharedPtr msg) { on_raw_command(msg); });

  RCLCPP_INFO(get_logger(), "Connecting to serial: '%s'", serial_port.c_str());

  try {
    // create a serial device with immediate timeouts.
    serial_ = std::make_unique<serial::Serial>(serial_port, BAUDRATE);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), e.what());
    throw;
  }
}

void RoverSerial::on_raw_command(openrover_core_msgs::msg::RawCommand::SharedPtr cmd)
{
  auto efforts = *motor_efforts_u8;
  std::vector<uint8_t> payload{efforts[0], efforts[1], efforts[2], cmd->verb, cmd->arg};
  auto packed = packetize(payload);
  this->serial_->write(packed);
  keepalive_timer->reset();
}

void RoverSerial::keepalive_callback()
{
  RCLCPP_DEBUG(this->get_logger(), "begin keepalive");

  msg::RawCommand cmd;
  cmd.verb = 10;
  cmd.arg = 40;
  auto efforts = *motor_efforts_u8;
  std::vector<uint8_t> payload{efforts[0], efforts[1], efforts[2], cmd.verb, cmd.arg};
  auto packed = packetize(payload);
  this->serial_->write(packed);

  RCLCPP_DEBUG(this->get_logger(), "end keepalive");
}

std::string strhex(const std::vector<uint8_t> & bin)
{
  std::ostringstream ss;
  ss << std::hex << std::setw(2) << std::setfill('0');
  for (auto h : bin) {
    ss << h;
  }

  return ss.str();
}

void RoverSerial::read_callback()
{
  const size_t READ_PACKET_SIZE = 5;
  while (true) {
    std::vector<uint8_t> inbuf;
    while (inbuf.empty()) {
      if (serial_->available() < READ_PACKET_SIZE) {
        return;  // not enough data to possibly form a full packet
      }
      auto n = serial_->read(inbuf, 1);
      if (n == 0) {
        return;  // timed out
      }
      if (inbuf[0] != UART_START_PACKET) {
        RCLCPP_WARN(this->get_logger(), "Expected start byte, instead got: %0x", inbuf[0]);
        inbuf.clear();
      }
    }

    serial_->read(inbuf, READ_PACKET_SIZE - 1);
    std::string hexdata = strhex(inbuf);
    RCLCPP_DEBUG(this->get_logger(), "Packet: %s.", hexdata.c_str());

    auto payload = depacketize(inbuf);

    auto data = std::make_unique<msg::RawData>();
    data->which = payload[0];
    data->value = {payload[1], payload[2]};
    this->pub_raw_data->publish(std::move(data));
  }
}

void RoverSerial::on_kill_motors()
{
  motor_efforts_u8 = std::make_shared<std::array<uint8_t, 3>>(MOTOR_EFFORT_HALT);
}

void RoverSerial::on_motor_efforts(openrover_core_msgs::msg::RawMotorCommand::SharedPtr msg)
{
  auto new_efforts = std::array<uint8_t, 3>{msg->left, msg->right, msg->flipper};
  motor_efforts_u8 = std::make_shared<const std::array<uint8_t, 3>>(new_efforts);

  this->kill_motors_timer->reset();

  auto efforts = *motor_efforts_u8;
  std::vector<uint8_t> payload{efforts[0], efforts[1], efforts[2], 0, 0};
  auto packed = packetize(payload);
  RCLCPP_DEBUG(this->get_logger(), "Writing data: 0x%d", strhex(packed).c_str());

  auto n_written = this->serial_->write(packed);
  if (n_written < packed.size()) {
    RCLCPP_WARN(
      this->get_logger(), "Could not write all data. Only wrote %d/%d bytes", n_written,
      packed.size());
  }
}
