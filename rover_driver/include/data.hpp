#pragma once

#include <bitset>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rover_msgs/msg/raw_data.hpp"

namespace rover
{
namespace data
{
using rover_msgs::msg::RawData;
using std::string;
using RawValue = decltype(RawData::value);

static auto as_uint16(RawValue r) { return (uint16_t)(r[0] << 8 | r[1]); }
static auto as_int16(RawValue r) { return (int16_t)(r[0] << 8 | r[1]); }

template <unsigned char N, typename T>
struct KnownDataElement
{
  KnownDataElement() = delete;

  using Value = T;
  static const uint8_t Which = N;
  static uint8_t which() { return N; }
  // not implemented on purpose
  static Value decode(RawValue r);
};

struct UnknownDataElement
{
  uint8_t which;
  RawValue raw_value;

  UnknownDataElement(uint8_t which, RawValue raw_value) : which(which), raw_value(raw_value) {}

  std::string to_string() const
  {
    std::stringstream stream;
    stream << "0x" << std::setfill('0') << std::setw(4) << std::hex << raw_value[0] << raw_value[1];
    return stream.str();
  }
};
struct LeftMotorRPM : KnownDataElement<2, int16_t>
{
  static int16_t decode(rover::data::RawValue r) { return as_int16(r); }
};
struct RightMotorRPM : KnownDataElement<4, int16_t>
{
  static int16_t decode(rover::data::RawValue r) { return as_int16(r); }
};
struct LeftMotorEncoderState : KnownDataElement<14, int16_t>
{
  static int16_t decode(rover::data::RawValue r) { return as_int16(r); }
};
struct RightMotorEncoderState : KnownDataElement<16, int16_t>
{
  static int16_t decode(rover::data::RawValue r) { return as_int16(r); }
};
struct MotorTemperature1 : KnownDataElement<20, double>
{
  static Value decode(RawValue r) { return as_uint16(r) * 1.0; }
};
struct MotorTemperature2 : KnownDataElement<22, double>
{
  static Value decode(RawValue r) { return as_uint16(r) * 1.0; }
};

// firmware 1.0 : BE CAREFUL reading the firmware code. It's messy and confusing
// 1:256 prescale with 16MHz base clock = 256 / (16 MHz) = 16 microseconds
// / 2 to compensate for only counting every *other* commutation
constexpr auto ENCODER_TIME_BASE = 256.0 / 16.0e6 / 2;
// the time it takes for a single commutation event
struct LeftMotorEncoderPeriod : KnownDataElement<28, double>
{
  static Value decode(RawValue r) { return as_uint16(r) * ENCODER_TIME_BASE; }
};
struct RightMotorEncoderPeriod : KnownDataElement<30, double>
{
  static Value decode(RawValue r) { return as_uint16(r) * ENCODER_TIME_BASE; }
};
struct FlipperMotorEncoderPeriod : KnownDataElement<32, double>
{
  static Value decode(RawValue r) { return as_uint16(r) * ENCODER_TIME_BASE; }
};
struct BatteryAStateOfCharge : KnownDataElement<34, double>
{
  static Value decode(RawValue r) { return as_uint16(r) / 100.0; }
};
struct BatteryBStateOfCharge : KnownDataElement<36, double>
{
  static Value decode(RawValue r) { return as_uint16(r) / 100.0; }
};
struct BatteryChargingState : KnownDataElement<38, bool>
{
  static Value decode(RawValue r) { return as_uint16(r) == 0xdada; }
};
struct RoverVersionValue
{
  int major;
  int minor;
  int patch;
  std::string to_string() const
  {
    if (major == 0 && minor == 0 && patch == 0) {
      return "legacy";
    } else {
      return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
  }
};

struct RoverVersion : KnownDataElement<40, RoverVersionValue>
{
  static Value decode(RawValue r)
  {
    auto v = as_uint16(r);
    if (v == 40621) {
      return {0, 0, 0};
    } else {
      return {v / 10000, v / 100 % 100, v / 1 % 100};
    }
  }
};

struct BatteryACurrent : KnownDataElement<42, double>
{
  static Value decode(RawValue r) { return as_uint16(r) / 34.0; }
};
struct BatteryBCurrent : KnownDataElement<44, double>
{
  static Value decode(RawValue r) { return as_uint16(r) / 34.0; }
};
struct BatteryACurrentInternal : KnownDataElement<68, double>
{
  static Value decode(RawValue r) { return as_int16(r) / 1000.0; }
};
struct BatteryBCurrentInternal : KnownDataElement<70, double>
{
  static Value decode(RawValue r) { return as_int16(r) / 1000.0; }
};

enum class MotorStatusFlag {
  /// No flags at all.
  MOTOR_FLAG_NONE = 0,

  /// Feedback flag: Does motor experience high current? 1 indicates some sort of long-circuit
  /// condition
  MOTOR_FLAG_FAULT1 = 1u << 0u,

  /// Feedback flag: Does motor experience low current? 1 indicates some sort of  short-circuit
  /// condition
  MOTOR_FLAG_FAULT2 = 1u << 1u,

  /// Control flag: Should use fast current decay?
  /// Fast mode has higher dynamic response but worse for maintaining speed.
  /// Ignored when coasting or braking.
  MOTOR_FLAG_DECAY_MODE = 1u << 2u,

  /// Control flag: Should drive motor in reverse direction? (this is the motor direction clockwise
  /// or counterclockwise, NOT forward or backwards w.r.t. the rover's heading)
  /// Ignored when coasting or braking.
  MOTOR_FLAG_REVERSE = 1u << 3u,

  /// Control flag: Should brake motor? If enabled, PWM value is ignored.
  /// Ignored when coasting.
  MOTOR_FLAG_BRAKE = 1u << 4u,

  /// Control flag: Should coast motor? If enabled, PWM value is ignored.
  MOTOR_FLAG_COAST = 1u << 5u,
};

struct LeftMotorStatus : KnownDataElement<72, MotorStatusFlag>
{
  static Value decode(RawValue r) { return MotorStatusFlag(as_uint16(r)); }
};
struct RightMotorStatus : KnownDataElement<74, MotorStatusFlag>
{
  static Value decode(RawValue r) { return MotorStatusFlag(as_uint16(r)); }
};
struct FlipperMotorStatus : KnownDataElement<76, MotorStatusFlag>
{
  static Value decode(RawValue r) { return MotorStatusFlag(as_uint16(r)); }
};
struct CoolingFan1DutyFactor : KnownDataElement<78, float>
{
  static Value decode(RawValue r) { return as_uint16(r) / 240.0F; }
};
struct CoolingFan2DutyFactor : KnownDataElement<80, float>
{
  static Value decode(RawValue r) { return as_uint16(r) / 240.0F; }
};

}  // namespace data
}  // namespace rover
