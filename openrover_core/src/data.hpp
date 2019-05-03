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

// based on a 16 MHz timer with 1:8 prescale
// todo:
// so in units of 62.5 ns * 8 = 500 ns = 5e-7s
// or is it 1:256 prescale = 1.6e-5
constexpr auto ENCODER_TIME_BASE = 1.6e-5;
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
