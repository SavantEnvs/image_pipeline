#pragma once
#include <cstdint>
#include <string>
namespace sensor_msgs::msg
{
struct PointField
{
  static constexpr uint8_t INT8 = 1;
  static constexpr uint8_t UINT8 = 2;
  static constexpr uint8_t INT16 = 3;
  static constexpr uint8_t UINT16 = 4;
  static constexpr uint8_t INT32 = 5;
  static constexpr uint8_t UINT32 = 6;
  static constexpr uint8_t FLOAT32 = 7;
  static constexpr uint8_t FLOAT64 = 8;
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 0;
};
}  // namespace sensor_msgs::msg
