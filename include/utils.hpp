#ifndef UTILS_HPP
#define UTILS_HPP

#include <librealsense2/rs.hpp>

namespace utils {

struct rs_handler {
  rs2::frame_queue frame_q;
  rs2::pointcloud pc;
  rs2::pipeline pipe;
  rs2::align align;

  rs_handler() : frame_q(), pc(), pipe(), align(RS2_STREAM_COLOR) {}
};

struct rs_config {
  uint16_t height, width;
  uint16_t fov[2];
  float depth_scale;
  uint8_t fps;
  bool enable_imu = false;
};

struct rs_handler *setupRealsense(struct Config &config);
} // namespace utils

#endif
