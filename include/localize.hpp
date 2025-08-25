#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include <variant>
#include <librealsense2/hpp/rs_processing.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include "alloc.hpp"

namespace percep {
struct Config {
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
extern uint64_t REALSENSE_HANDLE_SIZE;
const char* getError(enum Error err);
auto yawFromPose(Eigen::Matrix<double, 4, 4> &pose) -> double;
struct RealsenseHandle;
auto getStatus(RealsenseHandle* handle) -> std::string;
auto
getPointFromPixel(RealsenseHandle* handle, float x, float y, double depth)
  -> Eigen::Vector3d;
auto localizationLoopAdjustmentRunning(RealsenseHandle* handle) -> bool;
auto localizationMappingEnabled(RealsenseHandle* handle) -> bool;
auto resetLocalization(RealsenseHandle* handle) -> void;
auto
setupRealsense(mem::Allocator& alloc, struct Config& config)
  -> struct RealsenseHandle*;
auto destroyHandle(struct RealsenseHandle* handle) -> Error;
auto
getPoints(RealsenseHandle* handle) -> std::variant<Error, rs2::points>;
auto getFrames(RealsenseHandle *handle)
    -> std::variant<Error, std::pair<cv::Mat, cv::Mat>>;
auto runLocalization(RealsenseHandle *handle, const void *rerun_rec_ptr)
    -> std::variant<Error, Eigen::Matrix<double, 4,4>>;
} // namespace percep
