// Fuzzes image_proc::debayerEdgeAware / debayerEdgeAwareWeighted (image_proc/src/edge_aware.cpp).
//
// Mirrors DebayerNode::imageCb (image_proc/src/debayer.cpp): the node only rejects width == 0,
// height == 0 or empty data before wrapping the message buffer in an 8-bit single-channel cv::Mat
// (raw step) and calling the edge-aware debayer into a contiguous 8UC3 output of the same size.
// Edge-aware debayering is only reachable for BAYER_GRBG8, hence CV_8UC1.
//
// Message contract (sensor_msgs/Image.msg): `step` is the full row length in bytes, so step >= width
// for a one-byte pixel, and `data` holds exactly step * height bytes. Inputs that cannot form such
// a message are rejected with -1.
#include <cstddef>
#include <cstdint>

#include <fuzzer/FuzzedDataProvider.h>
#include <opencv2/core/core.hpp>

#include "image_proc/edge_aware.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t * data, size_t size)
{
  FuzzedDataProvider fdp(data, size);
  const bool weighted = fdp.ConsumeBool();
  const uint32_t width = fdp.ConsumeIntegralInRange<uint32_t>(0, 512);
  const uint32_t height = fdp.ConsumeIntegralInRange<uint32_t>(0, 512);
  const uint32_t step = width + fdp.ConsumeIntegralInRange<uint32_t>(0, 16);

  const uint64_t data_bytes = static_cast<uint64_t>(step) * height;
  if (data_bytes > fdp.remaining_bytes()) {
    return -1;
  }
  std::vector<uint8_t> raw = fdp.ConsumeBytes<uint8_t>(data_bytes);

  // The node's own guard.
  if (width == 0 || height == 0 || raw.empty()) {
    return 0;
  }

  const cv::Mat bayer(height, width, CV_8UC1, raw.data(), step);
  std::vector<uint8_t> color_data(static_cast<size_t>(height) * width * 3);
  cv::Mat color(height, width, CV_8UC3, color_data.data(), static_cast<size_t>(width) * 3);

  if (weighted) {
    image_proc::debayerEdgeAwareWeighted(bayer, color);
  } else {
    image_proc::debayerEdgeAware(bayer, color);
  }
  return 0;
}
