#ifndef SLAM_HPP
#define SLAM_HPP

#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stella_vslam/config.h>
#include <stella_vslam/system.h>

#include "utils.hpp"

namespace slam {

// init this struct in main
struct slamHandle {
  std::shared_ptr<stella_vslam::config> slam_cfg;
  stella_vslam::system slam;

  slamHandle()
      : slam_cfg(std::make_shared<stella_vslam::config>("stellaconf.yaml")),
        slam(slam_cfg, "orb_vocab.fbow") {
    slam.startup();
  }
};

struct slamPose {
	float x; 
	float y; 
	float z;
	float yaw;
};

struct rawColorDepthPair {
  const void *colorFrame;
  const void *depthFrame;
  double timestamp;
};

struct RGBDFrame {
  cv::Mat color_cv;
  cv::Mat depth_cv;
  double timestamp_cv;
};

float yawfromPose(Eigen::Matrix<double, 4, 4> &pose);

std::string getStatus(slamHandle *handle);

struct RGBDFrame *getColorDepthPair(struct rawColorDepthPair *frame);

void resetLocalization(slamHandle *handle);

bool localizationLoopAdjustmentRunning(slamHandle *handle);

auto runLocalization(RGBDFrame *frame_cv, slamHandle *handle, const void *rec)
    -> Eigen::Matrix<double, 4, 4>;
} // namespace slam
#endif
