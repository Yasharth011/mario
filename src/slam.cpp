#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rsutil.h>
#include <limits>
#include <rerun.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/archetypes/view_coordinates.hpp>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>

#include "slam.hpp"

namespace slam {
double yawfromPose(Eigen::Matrix<double, 4, 4> &pose) {
  double yaws[2] = {0, 0};

  // Error : cannot find yaw
  if (pose(2, 0) == 1 or pose(2, 0) == -1)
    return std::numeric_limits<double>::infinity();

  double pitch_1 = -1 * asin(pose(2, 0));
  double pitch_2 = M_PI - pitch_1;

  yaws[0] = atan2(pose(1, 0) / cos(pitch_1), pose(0, 0) / cos(pitch_1));
  yaws[1] = atan2(pose(1, 0) / cos(pitch_2), pose(0, 0) / cos(pitch_2));

  return yaws[1] - M_PI_2;
}

std::string getStatus(slamHandle *handle) {
  return handle->slam.get_frame_publisher()->get_tracking_state();
}

void resetLocalization(slamHandle *handle) { handle->slam.request_reset(); }

bool localizationLoopAdjustmentRunning(slamHandle *handle) {
  return handle->slam.loop_BA_is_running();
}

struct RGBDFrame *getColorDepthPair(utils::rs_handler *handle,
                                    rs2::frameset &fs) {

  struct RGBDFrame *frame_cv = new RGBDFrame();

  auto aligned_frames = handle->align.process(fs);

  rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
  rs2::video_frame depth_frame = aligned_frames.get_depth_frame();

  auto color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3,
                          (void *)(color_frame.get_data()), cv::Mat::AUTO_STEP);
  cv::cvtColor(color_cv, color_cv, cv::COLOR_BGR2RGB);

  auto depth_cv = cv::Mat(cv::Size(640, 480), CV_16U,
                          (void *)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

  auto timestamp_cv = fs.get_timestamp();

  frame_cv->depth_cv = depth_cv;
  frame_cv->color_cv = color_cv;
  frame_cv->timestamp_cv = timestamp_cv;

  return frame_cv;
}

auto runLocalization(RGBDFrame *frame_cv, slamHandle *handle,
                     const void *rec)
    -> std::variant<utils::Error, Eigen::Matrix<double, 4, 4>> {

  std::variant<utils::Error, Eigen::Matrix<double, 4, 4>> result;

  while (handle->slam.loop_BA_is_running() or
         (!handle->slam.mapping_module_is_enabled())) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  handle->slam.feed_RGBD_frame(frame_cv->color_cv, frame_cv->depth_cv,
                               frame_cv->timestamp_cv);
  auto pose = handle->slam.get_map_publisher()->get_current_cam_pose();

  Eigen::Matrix<double, 4, 4> inverse_pose = pose.inverse();

  auto frame = handle->slam.get_frame_publisher()->draw_frame();

  std::string state = handle->slam.get_frame_publisher()->get_tracking_state();

  if (state == "Tracking") {
    auto translations = inverse_pose.col(3);
    auto rotations = inverse_pose.block<3, 3>(0, 0);
  }
  result.emplace<1>(inverse_pose);

  return result;
}
} // namespace slam
#ifdef SLAM_TEST_CPP
#include <iostream>
#include <rerun.hpp>

int main() {

  struct utils::rs_config realsense_config{.height = 640,
                                           .width = 480,
                                           .fov = {0, 0},
                                           .depth_scale = 0.0,
                                           .fps = 30,
                                           .enable_imu = false};

  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");

  struct utils::rs_handler *rs_ptr = utils::setupRealsense(realsense_config);

  struct slamHandle *slam_handler = new slam::slamHandle();

  while (true) {
    rs2::frame frame = rs_ptr->frame_q.wait_for_frame();

    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      struct RGBDFrame *frame_cv = slam::getColorDepthPair(rs_ptr, fs);

      if (auto res = runLocalization(frame_cv, slam_handler, &rec);
          std::get_if<utils::Error>(&res)) {
        std::cout << "getFrames error: " << utils::Error(std::get<0>(res))
                  << std::endl;
      }
    }
  }
}
#endif
