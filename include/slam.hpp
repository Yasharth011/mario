#ifndef SLAM_HPP
#define SLAM_HPP
#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <stella_vslam/config.h>
#include <stella_vslam/system.h>

#include "localize.hpp"
#include "mario.hpp"

namespace slam {
// init this struct in main
struct slamHandle {
  stella_vslam::system slam;
  stella_vslam::config slam_cfg;
  slamHandle()
      : slam_cfg("stellaconf.yaml"),
        slam(std::shared_ptr<stella_vslam::config>(&slam_cfg, [](auto p) {}), 
             "stellavocab.txt") {
    slam.startup();
  }
};

// new (&handle->slam_cfg) stella_vslam::config("stellaconf.yaml");
// new (&handle->slam) stella_vslam::system(
//   std::shared_ptr<stella_vslam::config>(&handle->slam_cfg, [](auto p) {}),
//   "stellavocab.txt");
enum Error : uint8_t {
  NoError = 0,
  NoDeviceConnected,
  InvalidHandle,
  NoFrameset,
};

double yawfromPose(Eigen::Matrix<double, 4, 4> &pose);

std::string getStatus(slamHandle *handle);

Eigen::Vector3d getPointFromPixel(float x, float y, double depth);

auto getColorDepthPair(mario::rsHandle *handle);

auto getPoints(mario::rsHandle *handle) -> std::variant<Error, rs2::points>;

auto getFrames(mario::rsHandle *handle)
    -> std::variant<Error, std::pair<cv::Mat, cv::Mat>>;

void resetLocalization(slamHandle *handle);

bool localizationLoopAdjustmentRunning(slamHandle *handle);

auto runLocalization(rs::frame frame, slamHandle *handle, const void *rec);
} // namespace slam
#endif
