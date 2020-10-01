#include "rover_serial.hpp"
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

using namespace rover_msgs;
using namespace rover;

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
    throw roverError("Bad packet: did not start with a start byte");
  }

  std::vector<uint8_t> payload;
  for (size_t i = 1; i < packet.size() - 1; i++) {
    payload.push_back(packet[i]);
  }

  auto expected_checksum = checksum(payload);
  auto received_checksum = packet[packet.size() - 1];
  if (expected_checksum != received_checksum) {
    throw roverError("Bad packet: incorrect checksum");
  }

  return payload;
}

RoverSerial::RoverSerial()
: Node("rover_serial", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Starting rover serial communication node");

  serial_port = declare_parameter("serial_port", "/dev/ttyUSB0");

  motor_efforts_u8 = std::make_shared<std::array<uint8_t, 3>>(MOTOR_EFFORT_HALT);

  keepalive_timer = create_wall_timer(keepalive_period, [=]() { keepalive_callback(); });
  read_timer = create_wall_timer(uart_poll_period, [=]() { read_callback(); });

  // auto writer_group =
  // this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  kill_motors_timer = create_wall_timer(kill_motors_timeout, [=]() { on_kill_motors(); });

  pub_raw_data = create_publisher<rover_msgs::msg::RawData>("raw_data", rclcpp::QoS(16));

  sub_motor_efforts = create_subscription<msg::RawMotorCommand>(
    "motor_efforts", rclcpp::QoS(1),
    [=](msg::RawMotorCommand::SharedPtr msg) { on_motor_efforts(msg); });
  sub_raw_commands = create_subscription<msg::RawCommand>(
    "rover_command", rclcpp::QoS(16),
    [=](msg::RawCommand::SharedPtr msg) { on_raw_command(msg); });

  RCLCPP_INFO(get_logger(), "Connecting to serial: '%s'", serial_port.c_str());

  try {
    // create a serial device with immediate timeouts.
    open_serial();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), e.what());
    throw;
  }
}

void RoverSerial::on_raw_command(rover_msgs::msg::RawCommand::SharedPtr cmd)
{
  auto efforts = *motor_efforts_u8;
  std::vector<uint8_t> payload{efforts[0], efforts[1], efforts[2], cmd->verb, cmd->arg};
  auto packed = packetize(payload);
  serial_write(&packed[0], packed.size());
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
  serial_write(&packed[0], packed.size());

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
      // RCLCPP_WARN(this->get_logger(), "Available bytes: %d", serial_buffer_availible());
      if (serial_buffer_availible() < READ_PACKET_SIZE) {
        return;  // not enough data to possibly form a full packet
      }

      auto n = serial_read(inbuf, 1);
      if (n == 0) {
        return;  // timed out
      }
      
      if (inbuf[0] != UART_START_PACKET) {
        RCLCPP_WARN(this->get_logger(), "Expected start byte, instead got: %0x", inbuf[0]);
        inbuf.clear();
      }

    }

    serial_read(inbuf, READ_PACKET_SIZE - 1);

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

void RoverSerial::on_motor_efforts(rover_msgs::msg::RawMotorCommand::SharedPtr msg)
{
  auto new_efforts = std::array<uint8_t, 3>{msg->left, msg->right, msg->flipper};
  motor_efforts_u8 = std::make_shared<const std::array<uint8_t, 3>>(new_efforts);

  this->kill_motors_timer->reset();

  auto efforts = *motor_efforts_u8;
  std::vector<uint8_t> payload{efforts[0], efforts[1], efforts[2], 0, 0};
  auto packed = packetize(payload);
  RCLCPP_DEBUG(this->get_logger(), "Writing data: 0x%d", strhex(packed).c_str());

  auto n_written = serial_write(&packed[0], packed.size());
  if (n_written < packed.size()) {
    RCLCPP_WARN(
      this->get_logger(), "Could not write all data. Only wrote %d/%d bytes", n_written,
      packed.size());
  }
}

bool RoverSerial::open_serial()
{
  RCLCPP_INFO(this->get_logger(), "Opening serial port");
  struct termios serial_fd__options;

  serial_fd_ = std::make_shared<int>(::open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY));
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", strerror(errno));
    return false;
  }
  if (0 > fcntl(*serial_fd_, F_SETFL, 0))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set port descriptor: %s", strerror(errno));
    return false;
  }
  if (0 > tcgetattr(*serial_fd_, &serial_fd__options))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to fetch port attributes: %s", strerror(errno));
    return false;
  }
  if (0 > cfsetispeed(&serial_fd__options, B57600))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set input baud: %s", strerror(errno));
    return false;
  }
  if (0 > cfsetospeed(&serial_fd__options, B57600))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set output baud: %s", strerror(errno));
    return false;
  }

  serial_fd__options.c_cflag |= (CREAD | CLOCAL | CS8);
  serial_fd__options.c_cflag &= ~(PARODD | CRTSCTS | CSTOPB | PARENB);
  serial_fd__options.c_iflag &= ~(IUCLC | IXANY | IMAXBEL | IXON | IXOFF | IUTF8 | ICRNL | INPCK);  // input modes
  serial_fd__options.c_oflag |= (NL0 | CR0 | TAB0 | BS0 | VT0 | FF0);
  serial_fd__options.c_oflag &=
      ~(OPOST | ONLCR | OLCUC | OLCUC | ONOCR | ONLRET | OFILL | OFDEL | NL1 | CR1 | CR2 | TAB3 | BS1 | VT1 | FF1);
  serial_fd__options.c_lflag |= (NOFLSH);
  serial_fd__options.c_lflag &= ~(ICANON | IEXTEN | TOSTOP | ISIG | ECHOPRT | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
  serial_fd__options.c_cc[VINTR] = 0x03;   // INTR Character
  serial_fd__options.c_cc[VQUIT] = 0x1C;   // QUIT Character
  serial_fd__options.c_cc[VERASE] = 0x7F;  // ERASE Character
  serial_fd__options.c_cc[VKILL] = 0x15;   // KILL Character
  serial_fd__options.c_cc[VEOF] = 0x04;    // EOF Character
  serial_fd__options.c_cc[VTIME] = 0x01;   // Timeout in 0.1s of serial read
  serial_fd__options.c_cc[VMIN] = 0;       // SERIAL_IN_PACKAGE_LENGTH; //Min Number of bytes to read
  serial_fd__options.c_cc[VSWTC] = 0x00;
  serial_fd__options.c_cc[VSTART] = UART_START_PACKET;  // START Character
  serial_fd__options.c_cc[VSTOP] = 0x13;                // STOP character
  serial_fd__options.c_cc[VSUSP] = 0x1A;                // SUSP character
  serial_fd__options.c_cc[VEOL] = 0x00;                 // EOL Character
  serial_fd__options.c_cc[VREPRINT] = 0x12;
  serial_fd__options.c_cc[VDISCARD] = 0x0F;
  serial_fd__options.c_cc[VWERASE] = 0x17;
  serial_fd__options.c_cc[VLNEXT] = 0x16;
  serial_fd__options.c_cc[VEOL2] = 0x00;

  if (0 > tcsetattr(*serial_fd_, TCSANOW, &serial_fd__options))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set port attributes: %s", strerror(errno));
    return false;
  }
  ::ioctl(*serial_fd_, TIOCEXCL);  // turn on exclusive mode

  RCLCPP_INFO(this->get_logger(), "Serial port opened");
  tcflush(*serial_fd_, TCIOFLUSH);  // flush received buffer

  return true;
}


unsigned int RoverSerial::serial_read(std::vector<uint8_t> &inbuf, size_t size)
{
  uint8_t *inbuf_ = new uint8_t[size];
  int bytes_read;

  try {
    bytes_read = ::read(*serial_fd_, inbuf_, size);
  }
  catch (const std::exception &e) {
    delete[] inbuf_;
    throw;
  }

  inbuf.insert(inbuf.end(), inbuf_, inbuf_+bytes_read);

  delete[] inbuf_;
  return bytes_read;
}

unsigned int RoverSerial::serial_write(const uint8_t *data, size_t length)
{
  unsigned int written_bytes = ::write(*serial_fd_, data, length);
  if ( written_bytes  < length)
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to send command serial command to rover");
    throw;
  }

  return written_bytes;
}

unsigned int RoverSerial::serial_buffer_availible(){
  int bytes_in_buffer;
  if (-1 == ::ioctl(*serial_fd_, FIONREAD, &bytes_in_buffer)) {
    throw;
  }
  return bytes_in_buffer;
}