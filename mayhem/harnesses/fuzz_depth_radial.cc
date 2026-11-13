// Fuzzes depth_image_proc::initMatrix / convertDepthRadial (depth_image_proc/src/conversions.cpp)
// as driven by PointCloudXyzRadialNode::depthCb (point_cloud_xyz_radial.cpp). That callback has no
// input guard of its own: it allocates the cloud from the depth image's width x height, rebuilds the
// radial transform from the CameraInfo K/D and size, and converts.
//
// Message contract (sensor_msgs/Image.msg): `data` holds exactly step * height bytes. A message that
// cannot be formed that way is rejected with -1; `step` covers a full row of pixels. The CameraInfo (K, D, width, height) arrives on a
// separate topic, so its size is independent of the depth image's, exactly as in the node.
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <fuzzer/FuzzedDataProvider.h>
#include <opencv2/core/core.hpp>

#include "depth_image_proc/conversions.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace enc = sensor_msgs::image_encodings;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  FuzzedDataProvider fdp(data, size);

  std::array<double, 9> k{};
  for (double & v : k) {
    v = fdp.ConsumeBool() ? fdp.ConsumeFloatingPoint<float>() : 0.0;
  }
  std::vector<double> d(fdp.PickValueInArray<size_t>({0, 4, 5, 8}));
  for (double & v : d) {
    v = fdp.ConsumeFloatingPoint<float>();
  }

  // Zero-sized CameraInfo or depth images are reachable only when this flag is set, so the empty
  // input (which libFuzzer runs first) stays a clean run instead of terminating on an OpenCV
  // exception / empty-vector dereference before anything is explored.
  const bool allow_degenerate = fdp.ConsumeBool();
  const uint32_t min_dim = allow_degenerate ? 0 : 1;
  const uint32_t info_width = fdp.ConsumeIntegralInRange<uint32_t>(min_dim, 64);
  const uint32_t info_height = fdp.ConsumeIntegralInRange<uint32_t>(min_dim, 64);

  auto depth = std::make_shared<sensor_msgs::msg::Image>();
  depth->width = fdp.ConsumeIntegralInRange<uint32_t>(min_dim, 64);
  depth->height = fdp.ConsumeIntegralInRange<uint32_t>(min_dim, 64);
  const bool is_float = fdp.ConsumeBool();
  depth->encoding = is_float ? enc::TYPE_32FC1 : enc::TYPE_16UC1;
  depth->step = depth->width * (is_float ? 4 : 2) + fdp.ConsumeIntegralInRange<uint32_t>(0, 16);
  const uint64_t bytes = static_cast<uint64_t>(depth->step) * depth->height;
  if (bytes > fdp.remaining_bytes()) {
    return -1;
  }
  depth->data = fdp.ConsumeBytes<uint8_t>(bytes);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = depth->height;
  cloud.width = depth->width;
  cloud.is_dense = false;
  cloud.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  const cv::Mat transform = depth_image_proc::initMatrix(
    cv::Mat_<double>(3, 3, k.data()), cv::Mat(d), info_width, info_height, true);

  if (is_float) {
    depth_image_proc::convertDepthRadial<float>(depth, cloud, transform);
  } else {
    depth_image_proc::convertDepthRadial<uint16_t>(depth, cloud, transform);
  }
  return 0;
}
