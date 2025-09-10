#ifndef SLAM_HPP
#define SLAM_HPP

#include <Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <stella_vslam/config.h>
#include <stella_vslam/system.h>
#include <variant>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "utils.hpp"

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

struct RGBDFrame{
	cv::Mat color_cv;
	cv::Mat depth_cv;
	double timestamp_cv;
};

double yawfromPose(Eigen::Matrix<double, 4, 4> &pose);

std::string getStatus(slamHandle *handle);

struct RGBDFrame *getColorDepthPair(utils::rs_handler *handle, rs2::frameset &fs);

void resetLocalization(slamHandle *handle);

bool localizationLoopAdjustmentRunning(slamHandle *handle);

auto runLocalization(RGBDFrame *frame_cv, slamHandle *handle, const void *rec)-> std::variant<utils::Error, Eigen::Matrix<double, 4, 4>> ;
} // namespace slam
#endif
