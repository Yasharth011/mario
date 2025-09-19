#ifndef UTILS_HPP
#define UTILS_HPP

#include <librealsense2/h/rs_types.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <zmq.hpp>

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

enum Error : uint8_t {
  NoError = 0,
  NoDeviceConnected,
  InvalidHandle,
  NoFrameset,
};

struct rs_handler *setupRealsense(struct rs_config config);

Error destroyHandle(struct utils::rs_handler *handle);

const char *get_error(enum Error err);

int publish_msg(zmq::socket_t &pub, const std::string &topic_name,
                std::function<zmq::message_t()> get_encoded_msg);
} // namespace utils
#endif
