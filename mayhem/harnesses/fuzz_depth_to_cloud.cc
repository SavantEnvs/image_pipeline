// Fuzzes depth_image_proc::convertDepth / convertRgb (depth_image_proc/{include,src}/conversions.*)
// as driven by PointCloudXyzNode::depthCb (point_cloud_xyz.cpp, mode 0) and
// PointCloudXyzrgbNode::imageCb (point_cloud_xyzrgb.cpp, mode 1), including their input guards.
//
// Message contract (sensor_msgs/Image.msg): `data` holds exactly step * height bytes. A message that
// cannot be formed that way is rejected with -1. Everything else (dimensions, step, encoding,
// camera intrinsics, invalid_depth) is attacker-controlled, as on the ROS topic.
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <fuzzer/FuzzedDataProvider.h>

#include "depth_image_proc/conversions.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace enc = sensor_msgs::image_encodings;
using sensor_msgs::msg::Image;

namespace
{
constexpr uint64_t kMaxPixels = 100ULL * 1000ULL * 1000ULL;  // the nodes' default `max_pixels`

std::shared_ptr<Image> consume_image(
  FuzzedDataProvider & fdp, const std::string & encoding, uint32_t width, uint32_t height,
  uint32_t step)
{
  auto img = std::make_shared<Image>();
  img->encoding = encoding;
  img->width = width;
  img->height = height;
  img->step = step;
  const uint64_t bytes = static_cast<uint64_t>(img->step) * height;
  if (bytes > fdp.remaining_bytes()) {
    return nullptr;
  }
  img->data = fdp.ConsumeBytes<uint8_t>(bytes);
  return img;
}
}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  FuzzedDataProvider fdp(data, size);
  const bool with_rgb = fdp.ConsumeBool();

  sensor_msgs::msg::CameraInfo info;
  const double fx = fdp.ConsumeFloatingPoint<float>();
  const double cx = fdp.ConsumeFloatingPoint<float>();
  const double fy = fdp.ConsumeFloatingPoint<float>();
  const double cy = fdp.ConsumeFloatingPoint<float>();
  info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
  info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // The node's `invalid_depth` parameter: a configured distance in metres (0 = disabled).
  const double invalid_depth = fdp.ConsumeBool() ? 0.0 : fdp.ConsumeFloatingPointInRange<double>(0.0, 60.0);

  const uint32_t width = fdp.ConsumeBool() ?
    fdp.ConsumeIntegral<uint32_t>() : fdp.ConsumeIntegralInRange<uint32_t>(0, 64);
  const uint32_t height = fdp.ConsumeIntegralInRange<uint32_t>(0, 64);
  const int depth_kind = fdp.ConsumeIntegralInRange<int>(0, 2);
  const std::string depth_encoding =
    depth_kind == 0 ? enc::TYPE_16UC1 : depth_kind == 1 ? enc::TYPE_32FC1 : enc::MONO16;
  Image::ConstSharedPtr depth_msg =
    consume_image(fdp, depth_encoding, width, height, fdp.ConsumeIntegralInRange<uint32_t>(0, 1024));
  if (!depth_msg) {
    return -1;
  }

  Image::ConstSharedPtr rgb_msg;
  if (with_rgb) {
    static const std::string kRgbEncodings[] = {
      enc::RGB8, enc::BGR8, enc::RGBA8, enc::BGRA8, enc::MONO8};
    static const uint32_t kRgbPixelBytes[] = {3, 3, 4, 4, 1};
    // The node returns early (after a resize) if the RGB size differs; same-size is the direct path.
    // The node never inspects the RGB buffer, so (per the Image.msg contract) it must be a valid
    // image: step covers a full row of pixels.
    const int which = fdp.ConsumeIntegralInRange<int>(0, 4);
    const uint64_t min_step = static_cast<uint64_t>(width) * kRgbPixelBytes[which];
    if (min_step > 4096) {
      return -1;
    }
    rgb_msg = consume_image(
      fdp, kRgbEncodings[which], width, height,
      static_cast<uint32_t>(min_step) + fdp.ConsumeIntegralInRange<uint32_t>(0, 16));
    if (!rgb_msg) {
      return -1;
    }
  }

  // Guards shared by PointCloudXyzNode::depthCb and PointCloudXyzrgbNode::imageCb.
  const uint64_t num_pixels = static_cast<uint64_t>(depth_msg->height) * depth_msg->width;
  if (num_pixels == 0 || num_pixels > kMaxPixels) {
    return 0;
  }
  size_t bytes_per_pixel = 0;
  if (depth_msg->encoding == enc::TYPE_16UC1 || (!with_rgb && depth_msg->encoding == enc::MONO16)) {
    bytes_per_pixel = sizeof(uint16_t);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    bytes_per_pixel = sizeof(float);
  } else {
    return 0;
  }
  const size_t expected_bytes = static_cast<size_t>(num_pixels) * bytes_per_pixel;
  if (depth_msg->data.size() < expected_bytes ||
    depth_msg->step < static_cast<uint32_t>(depth_msg->width * bytes_per_pixel))
  {
    return 0;
  }

  int red_offset = 0, green_offset = 0, blue_offset = 0, color_step = 1;
  if (with_rgb) {
    if (rgb_msg->encoding == enc::RGB8) {
      red_offset = 0; green_offset = 1; blue_offset = 2; color_step = 3;
    } else if (rgb_msg->encoding == enc::RGBA8) {
      red_offset = 0; green_offset = 1; blue_offset = 2; color_step = 4;
    } else if (rgb_msg->encoding == enc::BGR8) {
      red_offset = 2; green_offset = 1; blue_offset = 0; color_step = 3;
    } else if (rgb_msg->encoding == enc::BGRA8) {
      red_offset = 2; green_offset = 1; blue_offset = 0; color_step = 4;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = depth_msg->height;
  cloud.width = depth_msg->width;
  cloud.is_dense = false;
  cloud.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  if (with_rgb) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  if (cloud.data.size() < static_cast<size_t>(num_pixels) * cloud.point_step) {
    return 0;
  }

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(info);

  if (depth_msg->encoding == enc::TYPE_32FC1) {
    depth_image_proc::convertDepth<float>(depth_msg, cloud, model, invalid_depth);
  } else {
    depth_image_proc::convertDepth<uint16_t>(depth_msg, cloud, model, invalid_depth);
  }
  if (with_rgb) {
    depth_image_proc::convertRgb(rgb_msg, cloud, red_offset, green_offset, blue_offset, color_step);
  }
  return 0;
}
