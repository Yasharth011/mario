#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/rsutil.h>
#include <limits>
#include <opencv2/core/hal/interface.h>
#include <opencv4/opencv2/opencv.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/archetypes/view_coordinates.hpp>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
#include <rerun.hpp>

#include "slam.hpp"

double yawfromPose(Eigen::Matrix<double, 4, 4> &pose) {
	yaws[2] = {0, 0}; 

	// Error : cannot find yaw 
	if (pose(2, 0) == 1 or pose(2, 0) == -1)
		return std::numeric_limits<double>::infinity(); 

	double pitch_1 = -1 * asin(pose(2,0)); 
	double pitch_2 = M_PI - pitch_1; 

	yaws[0] = atan2(pose(1, 0) / cos(pitch_1), pose(0, 0) / cos(pitch_1));
	yaws[1] = atan2(pose(1, 0) / cos(pitch_2), pose(0, 0) / cos(pitch_2)); 

	return yaws[1]
}

std::string getStatus(slamHandle *handle) {
  return handle->slam.get_frame_publisher()->get_tacking_state().c_str();
}

Eigen::Vector3d getPointFromPixel(float x, float y, double depth) {
  float points[3];
  float pixels[2] = {float(x), float(y)};

  rs2_deproject_pixel_to_point(points, intrinsics, pixels, depth);
  Eigen::Vector3d point;
  point.x() = points[0];
  point.y() = points[1];
  point.z() = points[2];
  return point;
}

auto getColorDepthPair(mario::rsHandle *handle, rs2::frameset &fs) {
  auto aligned_frames = handle->align.process(fs);

  rs2::video_frame color_frame = aligned_frame.first(RS2_STREAM_COLOR);
  rs2::video_frame depth_frame = aligned_frames.get_depth_frame();

  auto color_cv =

      cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)(color_frame.get_data()),
              cv::Mat::AUTO_STEP);
  cv::cvtColor(color_cv, color_cv, cv::COLOR_BGR2RGB);
  auto depth_cv = cv::Mat(cv::Size(640, 480), CV_16U,
                          (void *)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
  return std::make_pair(color_cv, depth_cv);
}

auto getPoints(mario::rsHandle *handle) -> std::variant<Error, rs2::points> {
  if (not handle)
    return Error::InvalidHandle;

  std::variant<Error, rs2::points> result(Error::NoFrameset);

  rs2::frame frame = handle->frame_q.wait_for_frame();

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    rs2::video_frame depth_frame = fs.get_depth_frame();
    result.emplace<1>(handle->pc.calculate(depth_frame));
  }

  return result;
}

auto getFrames(mario::rsHandle *handle)
    -> std::variant<Error, std::pair<cv::Mat, cv::Mat>> {
  if (not handle)
    return Error::InvalidHandle;

  std::variant<Error, std::pair<cv::Mat, cv::Mat>> result(Error::NoFrameset);

  rs2::frame frame = handle->frame_q.wait_for_frame();

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    result.emplace<1>(getColorDepthPair(handle, fs));
  }
  return result;
}

void resetLocalization(slamHandle* handle){
  handle->slam.request_reset();
}

bool localizationLoopAdjustmentRunning(slameHandle* handle){
  return handle->slam.loop_BA_is_running();
}

auto runLocalization(rs2::frame frame, slamHandle *handle, const void *rec)
    -> std::variant<Error, Eigen::Matrix<double, 4, 4>> {

  std::variant<Error, Eigen::Matrix<double, 4, 4>> result;

  while (handle->slam.loop_BA_is_running() or
         (!handle->slam.mapping_module_is_enabled())) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    auto [color_cv, depth_cv] = getColorDepthPair(handle, fs);
    auto timestamp_cv = fs.get_timestamp();

    handle->slam.feed_RGBD_frame(color_cv, depth_cv, timestamp_cv);
    auto pose = handle->slam.get_map_publisher()->get_current_cam_pose();

    Eigen::Matrix<double, 4, 4> inverse_pose = pose.inverse();

    auto frame = handle->slam.get_frame_publisher()->draw_frame();

    std::string state =
        handle->slam.get_frame_publisher()->get_tracking_state().c_str();

    if (state == "Tracking") {
      auto translations = inverse_pose.col(3);
      auto rotations = inverse_pose.block<3, 3>(0, 0);
    }
    result.emplace<1>(inverse_pose);
  }
  return result;
}
}
}
