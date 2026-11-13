#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
namespace sensor_msgs::msg
{
struct PointCloud2
{
  using SharedPtr = std::shared_ptr<PointCloud2>;
  using ConstSharedPtr = std::shared_ptr<const PointCloud2>;
  std_msgs::msg::Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  std::vector<uint8_t> data;
  bool is_dense = false;
};
}  // namespace sensor_msgs::msg
