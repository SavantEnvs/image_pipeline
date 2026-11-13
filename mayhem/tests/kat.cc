// Known-answer tests for the library code the fuzz targets exercise: image_proc edge-aware
// debayering and depth_image_proc depth -> point cloud conversion. Each line printed is
// "PASS <name>" or "FAIL <name> ...", counted by mayhem/test.sh.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "depth_image_proc/conversions.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "image_proc/edge_aware.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace enc = sensor_msgs::image_encodings;

static void report(bool ok, const std::string & name, const std::string & detail = "")
{
  std::printf("%s %s %s\n", ok ? "PASS" : "FAIL", name.c_str(), ok ? "" : detail.c_str());
}

static bool near(double a, double b, double tol = 1e-5)
{
  return std::fabs(a - b) <= tol;
}

// GRBG mosaic of a flat colour: row 0 = G R G R ..., row 1 = B G B G ...
static cv::Mat grbg_flat(int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
  cv::Mat m(h, w, CV_8UC1);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      m.at<uint8_t>(y, x) = (y % 2 == 0) ? ((x % 2 == 0) ? g : r) : ((x % 2 == 0) ? b : g);
    }
  }
  return m;
}

static void test_debayer(bool weighted)
{
  const std::string tag = weighted ? "debayer_weighted" : "debayer";
  auto run = [&](const cv::Mat & bayer, cv::Mat & out) {
      out = cv::Mat(bayer.rows, bayer.cols, CV_8UC3, cv::Scalar(0, 0, 0));
      if (weighted) {
        image_proc::debayerEdgeAwareWeighted(bayer, out);
      } else {
        image_proc::debayerEdgeAware(bayer, out);
      }
    };

  // A flat colour must be reproduced exactly (interpolating constant planes). In row 0 the blue
  // channel at even x >= 2 is not checked: upstream's first-row loop reads blue from the green
  // site below-right (bayer_pixel[step + 1]) rather than the blue site directly below.
  cv::Mat out;
  run(grbg_flat(8, 8, 200, 100, 50), out);
  int bad = 0;
  for (int y = 0; y < out.rows; ++y) {
    for (int x = 0; x < out.cols; ++x) {
      const cv::Vec3b p = out.at<cv::Vec3b>(y, x);
      const bool skip_blue = (y == 0 && x >= 2 && x % 2 == 0);
      if (p[0] != 200 || p[1] != 100 || (!skip_blue && p[2] != 50)) {
        ++bad;
      }
    }
  }
  report(bad == 0, tag + "_flat_colour", std::to_string(bad) + " pixels differ from (200,100,50)");

  // Top-left pixel is taken straight from its neighbours: R = b[0][1], G = b[0][0], B = b[1][0].
  cv::Mat noisy(8, 8, CV_8UC1);
  for (int i = 0; i < 64; ++i) {
    noisy.at<uint8_t>(i / 8, i % 8) = static_cast<uint8_t>((i * 37 + 11) % 251);
  }
  run(noisy, out);
  const cv::Vec3b p = out.at<cv::Vec3b>(0, 0);
  report(
    p[0] == noisy.at<uint8_t>(0, 1) && p[1] == noisy.at<uint8_t>(0, 0) &&
    p[2] == noisy.at<uint8_t>(1, 0), tag + "_top_left_pixel",
    "got " + std::to_string(p[0]) + "," + std::to_string(p[1]) + "," + std::to_string(p[2]));

  // The measured green sites are copied through unchanged.
  const cv::Vec3b g11 = out.at<cv::Vec3b>(1, 1);
  report(g11[1] == noisy.at<uint8_t>(1, 1), tag + "_green_site_passthrough");
}

static sensor_msgs::msg::CameraInfo make_info(double f, double cx, double cy)
{
  sensor_msgs::msg::CameraInfo info;
  info.k = {f, 0, cx, 0, f, cy, 0, 0, 1};
  info.p = {f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  return info;
}

template<typename T>
static std::shared_ptr<sensor_msgs::msg::Image> make_depth(
  const char * encoding, uint32_t w, uint32_t h, const std::vector<T> & values)
{
  auto img = std::make_shared<sensor_msgs::msg::Image>();
  img->encoding = encoding;
  img->width = w;
  img->height = h;
  img->step = w * sizeof(T);
  img->data.resize(values.size() * sizeof(T));
  std::memcpy(img->data.data(), values.data(), img->data.size());
  return img;
}

static sensor_msgs::msg::PointCloud2 make_cloud(uint32_t w, uint32_t h, bool rgb)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = h;
  cloud.width = w;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  if (rgb) {
    mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    mod.setPointCloud2FieldsByString(1, "xyz");
  }
  return cloud;
}

static void test_convert_depth()
{
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(make_info(2.0, 0.5, 0.5));

  // 2x2 uint16 depth in millimetres; (0,1) is invalid (0).
  auto depth = make_depth<uint16_t>(enc::TYPE_16UC1, 2, 2, {1000, 2000, 0, 4000});
  auto cloud = make_cloud(2, 2, false);
  depth_image_proc::convertDepth<uint16_t>(depth, cloud, model);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  const float ex[4] = {-0.25f, 0.5f, 0.f, 1.f};
  const float ey[4] = {-0.25f, -0.5f, 0.f, 1.f};
  const float ez[4] = {1.f, 2.f, 0.f, 4.f};
  bool ok = true;
  std::string detail;
  for (int i = 0; i < 4; ++i, ++x, ++y, ++z) {
    if (i == 2) {
      if (!(std::isnan(*x) && std::isnan(*y) && std::isnan(*z))) {
        ok = false;
        detail += " pixel2_not_nan";
      }
    } else if (!(near(*x, ex[i]) && near(*y, ey[i]) && near(*z, ez[i]))) {
      ok = false;
      detail += " pixel" + std::to_string(i) + "=" + std::to_string(*x) + "," +
        std::to_string(*y) + "," + std::to_string(*z);
    }
  }
  report(ok, "convert_depth_uint16", detail);

  // Same scene as 32FC1 in metres, with an invalid_depth substitute for the NaN pixel (1.5 m).
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto fdepth = make_depth<float>(enc::TYPE_32FC1, 2, 2, {1.f, 2.f, nan, 4.f});
  auto fcloud = make_cloud(2, 2, false);
  depth_image_proc::convertDepth<float>(fdepth, fcloud, model, 1.5);
  sensor_msgs::PointCloud2ConstIterator<float> fx(fcloud, "x"), fy(fcloud, "y"), fz(fcloud, "z");
  ok = true;
  detail.clear();
  const float fex[4] = {-0.25f, 0.5f, -0.375f, 1.f};
  const float fey[4] = {-0.25f, -0.5f, 0.375f, 1.f};
  const float fez[4] = {1.f, 2.f, 1.5f, 4.f};
  for (int i = 0; i < 4; ++i, ++fx, ++fy, ++fz) {
    if (!(near(*fx, fex[i]) && near(*fy, fey[i]) && near(*fz, fez[i]))) {
      ok = false;
      detail += " pixel" + std::to_string(i) + "=" + std::to_string(*fx) + "," +
        std::to_string(*fy) + "," + std::to_string(*fz);
    }
  }
  report(ok, "convert_depth_float_invalid_depth", detail);
}

static void test_convert_rgb()
{
  auto rgb = std::make_shared<sensor_msgs::msg::Image>();
  rgb->encoding = enc::BGR8;
  rgb->width = 2;
  rgb->height = 2;
  rgb->step = 7;  // one padding byte per row
  rgb->data = {10, 20, 30, 40, 50, 60, 0, 70, 80, 90, 100, 110, 120, 0};
  auto cloud = make_cloud(2, 2, true);
  // BGR8: red at offset 2, green 1, blue 0, three bytes per pixel.
  depth_image_proc::convertRgb(rgb, cloud, 2, 1, 0, 3);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
  const uint8_t er[4] = {30, 60, 90, 120}, eg[4] = {20, 50, 80, 110}, eb[4] = {10, 40, 70, 100};
  bool ok = true;
  for (int i = 0; i < 4; ++i, ++r, ++g, ++b) {
    ok = ok && *r == er[i] && *g == eg[i] && *b == eb[i];
  }
  report(ok, "convert_rgb_bgr8_with_row_padding");
}

static void test_radial()
{
  const double f = 4.0, cx = 2.0, cy = 1.0;
  std::array<double, 9> k = {f, 0, cx, 0, f, cy, 0, 0, 1};
  const int w = 5, h = 3;
  cv::Mat transform = depth_image_proc::initMatrix(
    cv::Mat_<double>(3, 3, k.data()), cv::Mat(), w, h, true);
  report(transform.rows == w && transform.cols == h && transform.channels() == 3,
    "radial_matrix_shape");

  std::vector<uint16_t> mm(w * h, 3000);
  auto depth = make_depth<uint16_t>(enc::TYPE_16UC1, w, h, mm);
  auto cloud = make_cloud(w, h, false);
  depth_image_proc::convertDepthRadial<uint16_t>(depth, cloud, transform);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  bool unit_range = true, principal = true;
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u, ++x, ++y, ++z) {
      const double n = std::sqrt(double(*x) * *x + double(*y) * *y + double(*z) * *z);
      unit_range = unit_range && near(n, 3.0, 1e-4);
      if (u == 2 && v == 1) {
        principal = near(*x, 0) && near(*y, 0) && near(*z, 3.0);
      }
      if (u == 4 && v == 1) {  // x offset 2 px at f=4: direction (0.5, 0, 1)/|.|
        unit_range = unit_range && near(*x, 3.0 * 0.5 / std::sqrt(1.25), 1e-4);
      }
    }
  }
  report(unit_range, "radial_points_lie_at_measured_range");
  report(principal, "radial_principal_point_on_axis");
}

int main()
{
  test_debayer(false);
  test_debayer(true);
  test_convert_depth();
  test_convert_rgb();
  test_radial();
  return 0;
}
