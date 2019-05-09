#pragma once

#include "openrover_core_msgs/msg/raw_data.hpp"
#include <bitset>
#include <functional>
#include <iomanip>
#include <string>
#include <sstream>
#include "diagnostic_msgs/msg/key_value.hpp"

namespace openrover
{
namespace data
{

using openrover_core_msgs::msg::RawData;
using std::string;
using RawValue = decltype(RawData::value);

static auto as_uint16(RawValue r) { return ((uint16_t)r[0]) << 8 | (uint16_t)r[1]; }
static auto as_int16(RawValue r) { return ((int16_t)r[0]) << 8 | (int16_t)r[1]; }
static auto as_bits(RawValue r) { return std::bitset<16>(as_uint16(r)); }

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

struct LeftMotorEncoderState : KnownDataElement<14, int16_t>
{
  static int16_t decode(openrover::data::RawValue r) { return as_int16(r); }
};
struct RightMotorEncoderState : KnownDataElement<16, int16_t>
{
  static int16_t decode(openrover::data::RawValue r) { return as_int16(r); }
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

struct RoverVersionValue
{
  int major;
  int minor;
  int patch;
  std::string to_string() const
  {
    if (major == 0 && minor == 0 && patch == 0)
    {
      return "legacy";
    }
    else
    {
      return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
  }
};

struct RoverVersion : KnownDataElement<40, RoverVersionValue>
{
  static Value decode(RawValue r)
  {
    auto v = as_uint16(r);
    if (v == 40621)
      return { 0, 0, 0 };
    else
      return { v / 10000, v / 100 % 100, v / 1 % 100 };
  }
};

}  // namespace data
}  // namespace openrover
