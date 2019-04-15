#include <chrono>
#include <sstream>
#include <iostream>
#include <iomanip>
#include "connection.hpp"
using std::placeholders::_1;
using namespace openrover_core_msgs;
using namespace openrover;

const std::array<uint8_t, 3> MOTOR_EFFORT_HALT = { 125, 125, 125 };

uint8_t checksum(std::vector<uint8_t> payload)
{
  uint32_t sum = 0;
  for (auto i : payload)
  {
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
  if (packet[0] != UART_START_PACKET)
    throw new OpenRoverError("Bad packet: did not start with a start byte");

  std::vector<uint8_t> payload;
  for (auto i = 1; i < packet.size() - 1; i++)
  {
    payload.push_back(packet[i]);
  }

  auto expected_checksum = checksum(payload);
  auto received_checksum = packet[packet.size() - 1];
  if (expected_checksum != received_checksum)
    throw new OpenRoverError("Bad packet: incorrect checksum");

  return payload;
}

using Cls = Connection;
Connection::Connection() : Node("openrover_serial", "", true), motor_efforts_u8(MOTOR_EFFORT_HALT)
{
  RCLCPP_INFO(this->get_logger(), "starting node OpenRoverSerial");
  std::string port = get_parameter("port").as_string();
  connect(port);
}

Connection::Connection(std::string port) : Node("openrover_serial", "", true), motor_efforts_u8(MOTOR_EFFORT_HALT)
{
  RCLCPP_INFO(this->get_logger(), "starting node OpenRoverSerial");
}
void Connection::connect(std::string port)
{
  RCLCPP_INFO(this->get_logger(), "\tport = %s ", port);
  keepalive_timer = this->create_wall_timer(keepalive_period, std::bind(&Cls::keepalive_callback, this));
  read_timer = this->create_wall_timer(uart_poll_period, std::bind(&Cls::read_callback, this));

  // auto writer_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  kill_motors_timer = this->create_wall_timer(kill_motors_timeout, std::bind(&Cls::on_kill_motors, this));

  pub_raw_data = this->create_publisher<openrover_core_msgs::msg::RawData>("raw_data");

  sub_motor_efforts = this->create_subscription<msg::RawMotorCommand>(
      "motor_speeds_commanded", std::bind(&Cls::on_motor_speed_commanded, this, _1), 1);  //, writer_group);
  sub_raw_commands = this->create_subscription<msg::RawCommand>(
      "raw_command", std::bind(&Cls::on_raw_command, this, _1), 16);  // , writer_group);

  RCLCPP_INFO(this->get_logger(), "Connecting to serial: '%s'", port.c_str());

  try
  {
    // create a serial device with immediate timeouts.
    serial = std::make_unique<serial::Serial>(port, BAUDRATE);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(), e.what());
    throw;
  }
}

std::vector<std::string> Connection::list_ftdi_ports()
{
  std::vector<std::string> ports;

  auto ps = serial::list_ports();
  for (auto& p : ps)
  {
    if (p.hardware_id.find("FTDI") < p.hardware_id.size())
      ports.push_back(p.port);
  }
  return ports;
}

void Connection::on_raw_command(openrover_core_msgs::msg::RawCommand::SharedPtr cmd)
{
  auto efforts = motor_efforts_u8.load();
  std::vector<uint8_t> payload{ efforts[0], efforts[1], efforts[2], cmd->verb, cmd->arg };
  auto packed = packetize(payload);
  this->serial->write(packed);
  keepalive_timer->reset();
}

void Connection::keepalive_callback()
{
  msg::RawCommand cmd;
  cmd.verb = 10;
  cmd.arg = 40;
  auto efforts = motor_efforts_u8.load();
  std::vector<uint8_t> payload{ efforts[0], efforts[1], efforts[2], cmd.verb, cmd.arg };
  auto packed = packetize(payload);
  this->serial->write(packed);
}

std::string strhex(std::vector<uint8_t> bin)
{
  std::ostringstream ss;
  ss << std::hex << std::setw(2) << std::setfill('0');
  for (auto h : bin)
  {
    ss << h;
  }

  return ss.str();
}

void Connection::read_callback()
{
  const size_t READ_PACKET_SIZE = 5;
  while (true)
  {
    std::vector<uint8_t> inbuf;
    while (inbuf.size() == 0)
    {
      if (serial->available() < READ_PACKET_SIZE)
        return;  // not enough data to possibly form a full packet

      auto n = serial->read(inbuf, 1);
      if (n == 0)
        return;  // timed out

      if (inbuf[0] != UART_START_PACKET)
      {
        RCLCPP_WARN(this->get_logger(), "Expected start byte, instead got: %0x", inbuf[0]);
        inbuf.clear();
      }
    }

    auto t = serial->read(inbuf, READ_PACKET_SIZE - 1);

    RCLCPP_DEBUG(this->get_logger(), "Packet: %s.", strhex(inbuf));

    auto payload = depacketize(inbuf);

    msg::RawData data;
    data.which = payload[0];
    data.value = { payload[1], payload[2] };

    this->pub_raw_data->publish(data);
  }
}

void Connection::on_kill_motors() { motor_efforts_u8 = MOTOR_EFFORT_HALT; }

void Connection::on_motor_speed_commanded(openrover_core_msgs::msg::RawMotorCommand::SharedPtr msg)
{
  this->motor_efforts_u8 = { msg->left, msg->right, msg->flipper };
  this->kill_motors_timer->reset();
}
