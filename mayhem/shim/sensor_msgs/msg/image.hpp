#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "std_msgs/msg/header.hpp"
namespace sensor_msgs::msg
{
struct Image
{
  using SharedPtr = std::shared_ptr<Image>;
  using ConstSharedPtr = std::shared_ptr<const Image>;
  std_msgs::msg::Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string encoding;
  uint8_t is_bigendian = 0;
  uint32_t step = 0;
  std::vector<uint8_t> data;
};
}  // namespace sensor_msgs::msg
