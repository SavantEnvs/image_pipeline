#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "std_msgs/msg/header.hpp"
namespace sensor_msgs::msg
{
struct CameraInfo
{
  using SharedPtr = std::shared_ptr<CameraInfo>;
  using ConstSharedPtr = std::shared_ptr<const CameraInfo>;
  std_msgs::msg::Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k{};
  std::array<double, 9> r{};
  std::array<double, 12> p{};
  uint32_t binning_x = 0;
  uint32_t binning_y = 0;
  RegionOfInterest roi;
};
}  // namespace sensor_msgs::msg
