#include "openrover_core_msgs/msg/raw_data.hpp"
#include <bitset>
#include <functional>
using openrover_core_msgs::msg::RawData;
namespace openrover
{
using RawValue = decltype(RawData::value);

auto as_uint16(RawValue r) {return ((uint16_t)r[0]) << 8 | (uint16_t)r[1];}
auto as_int16(RawValue r) {return ((int16_t)r[0]) << 8 | (int16_t)r[1];}
auto as_bits(RawValue r) {return std::bitset<16>(as_uint16(r));}

std::function<float(RawValue)> scale(float factor, std::function<float(RawValue)> f)
{
  return [ = ](auto v) {return factor * f(v);};
}

std::function<float(RawValue)> offset(float x, std::function<float(RawValue)> f)
{
  return [ = ](auto v) {return x + f(v);};
}

template<typename T>
std::function<float(RawValue)> casted(std::function<float(RawValue)> f)
{
  return [ = ](auto v) {static_cast<T>(f(v));};
}

int add(int a, int b) {return a + b;}

float as_unsigned_over_34(RawValue r) {return as_uint16(r) / 34.0f;}

template<unsigned char N, typename T>
T deserialize(RawValue);

template<unsigned char N>
struct DataDesc
{
  static_assert(false);
};

template<typename T>
struct DataDescriptor
{
  template<typename F>
  DataDescriptor(const char * description, F f)
  : description(description), deserialize(f) {}
  using type = T;
  std::string description;
  std::function<T(RawValue)> deserialize;
};

template<unsigned char N, typename T>
struct BaseDesc
{
  static constexpr uint8_t Which = N;
  uint8_t which() {return N;}
};

template<unsigned char N, typename T>
T get_descriptor();

struct DataDesc<0>: BaseDesc<0, float>
{
  static auto description() {return "Total supply current in amperes";}
  static auto deserialize(RawValue r) {return as_uint16(r) / 34.0;}
};

struct DataDesc<42>: BaseDesc<42, float>
{
  static auto description() {return "Battery A supplied current in amperes";}
  static auto deserialize(RawValue r) {return as_uint16(r) / 34.0;}
};
struct DataDesc<44>
{
  static auto description() {return "Battery B supplied current in amperes";}
  static auto deserialize(RawValue r) {return as_uint16(r) / 34.0;}
};
struct DataDesc<10>
{
  static auto description() {return "Left motor current in amperes";}
  static auto deserialize(RawValue r) {return as_uint16(r) / 34.0;}
};
template<>
DataDescriptor<float> get_descriptor<12>()
{
  return {"Right motor current in amperes", scale(1 / 34.0, as_uint16)};
}

template<>
DataDescriptor<float> get_descriptor<20>()
{
  return {"Motor temperature (celcius)", as_uint16};
}

enum class MotorStatusFlag
{
  NONE = 0,
  FAULT1 = 1 << 0,
  FAULT2 = 1 << 1,
  DECAY_MODE = 1 << 2,
  REVERSE = 1 << 3,
  BRAKE = 1 << 4,
  COAST = 1 << 5,
};

template<>
DataDescriptor<MotorStatusFlag> get_descriptor<72>()
{
  return {"Left motor status flags", casted<MotorStatusFlag>(as_uint16)};
}

auto x = get_descriptor<72>();

}  // namespace openrover
