#pragma once
#include <cstdint>
namespace sensor_msgs::msg
{
struct RegionOfInterest
{
  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t height = 0;
  uint32_t width = 0;
  bool do_rectify = false;
};
}  // namespace sensor_msgs::msg
