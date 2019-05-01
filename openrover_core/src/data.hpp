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

struct BaseDataElement
{
  virtual std::string string_value() const = 0;
};

template <unsigned char N, typename T>
struct KnownDataElement : public BaseDataElement
{
  static uint8_t which() { return N; }
  RawValue raw_value;
  virtual T get_value() const = 0;
  virtual std::string string_value() const { return std::to_string(get_value()); }

  KnownDataElement(RawValue raw_value) : raw_value(raw_value){};
};
struct UnknownDataElement : public BaseDataElement
{
  uint8_t _which;
  RawValue raw_value;
  uint8_t which() const { return _which; }

  UnknownDataElement(uint8_t which, RawValue raw_value) : _which(which), raw_value(raw_value) {}
  virtual std::string string_value() const override
  {
    std::stringstream stream;
    stream << "0x" << std::setfill('0') << std::setw(4) << std::hex << raw_value[0] << raw_value[1];
    return stream.str();
  }
};
struct LeftMotorEncoderState : KnownDataElement<14, int16_t>
{
  using KnownDataElement::KnownDataElement;
  int16_t get_value() const { return as_int16(raw_value); }
};
struct RightMotorEncoderState : KnownDataElement<16, int16_t>
{
  using KnownDataElement::KnownDataElement;
  int16_t get_value() const { return as_int16(raw_value); }
};
struct FlipperMotorEncoderState : KnownDataElement<32, int16_t>
{
  using KnownDataElement::KnownDataElement;
  int16_t get_value() const { return as_int16(raw_value); }
};
struct RoverVersion : KnownDataElement<40, uint16_t>
{
  using KnownDataElement::KnownDataElement;
  uint16_t get_value() const
  {
    auto value = as_uint16(raw_value);
    return value == 40621 ? 0 : value;
  }
  virtual std::string string_value() const override
  {
    auto v = get_value();
    if (v == 0)
      return "legacy";

    auto major = v / 10000;
    auto minor = v / 100 % 100;
    auto patch = v / 1 % 100;

    std::stringstream stream;
    stream << major << "." << minor << "." << patch;
    return stream.str();
  }
};

}  // namespace data
}  // namespace openrover