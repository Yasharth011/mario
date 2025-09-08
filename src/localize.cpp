#include <chrono>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/rsutil.h>
#include <opencv4/opencv2/opencv.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/archetypes/view_coordinates.hpp>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
#include <rerun.hpp>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <thread>

#include "alloc.hpp"
#include "cppturbo.hpp"
#include "collection_adapters.hpp"
#include "localize.hpp"

template <> struct fmt::formatter<Eigen::Matrix4d> : ostream_formatter {};

auto yawFromPose(Eigen::Matrix<double, 4, 4> &pose) -> double {
  std::array<double, 2> yaws = {0, 0};
  if (pose(2, 0) == 1 or pose(2, 0) == -1) {
    printf("Pitch (θ) = ±90°!. Cannot find yaw!\n");
    return std::numeric_limits<double>::infinity();
  }
  double pitch_1 = -1 * asin(pose(2, 0));
  double pitch_2 = M_PI - pitch_1;
  yaws[0] = atan2(pose(1, 0) / cos(pitch_1), pose(0, 0) / cos(pitch_1));
  yaws[1] = atan2(pose(1, 0) / cos(pitch_2), pose(0, 0) / cos(pitch_2));
  // printf("%3.3f %3.3f\n", yaws[0]*180/M_PI, yaws[1]*180/M_PI);
  // auto min_yaw = std::min_element(
  //     yaws.cbegin(), yaws.cend(),
  //     [](const double &a, const double &b) { return fabs(a) < fabs(b); });
  return yaws[1] - M_PI_2;
  // return yaws;
}

};


uint64_t REALSENSE_HANDLE_SIZE = sizeof(struct RealsenseHandle);


// get status of slam
auto getStatus(RealsenseHandle* handle) -> std::string {
  return handle->slam.get_frame_publisher()->get_tracking_state().c_str();
}

// setup realsense and start SLAM
auto
setupRealsense(mem::Allocator& alloc, struct Config& config)
  -> struct RealsenseHandle*
{
  struct RealsenseHandle* handle = reinterpret_cast<struct RealsenseHandle*>(
    alloc.alloc(sizeof(struct RealsenseHandle)));

  new (&handle->pipe) rs2::pipeline();
  new (&handle->frame_q) rs2::frame_queue(); 
  new (&handle->pc) rs2::pointcloud(); 
  new (&handle->align) rs2::align(RS2_STREAM_COLOR);
  new (&handle->slam_cfg) stella_vslam::config("stellaconf.yaml");
  new (&handle->slam) stella_vslam::system(
    std::shared_ptr<stella_vslam::config>(&handle->slam_cfg, [](auto p) {}),
    "stellavocab.txt");

  handle->slam.startup();

  rs2::config stream_config;
  rs2::context ctx;
  float fov[2];

  auto devices = ctx.query_devices();
  if (devices.size() == 0) return nullptr;
  stream_config.enable_stream(rs2_stream::RS2_STREAM_COLOR,
                              0,
                              config.height,
                              config.width,
                              rs2_format::RS2_FORMAT_BGR8,
                              config.fps);
  stream_config.enable_stream(rs2_stream::RS2_STREAM_DEPTH,
                              0,
                              config.height,
                              config.width,
                              rs2_format::RS2_FORMAT_Z16,
                              config.fps);
  if (config.enable_imu) {
    stream_config.enable_stream(rs2_stream::RS2_STREAM_ACCEL,
                                RS2_FORMAT_MOTION_XYZ32F);
    stream_config.enable_stream(rs2_stream::RS2_STREAM_GYRO,
                                RS2_FORMAT_MOTION_XYZ32F);
  }
  rs2::pipeline_profile selection = handle->pipe.start(stream_config, handle->frame_q);
  // Get intrinsics from the stream: Depth Scale and FOV
  config.depth_scale = selection.get_device()
                           .query_sensors()
                           .front()
                           .as<rs2::depth_sensor>()
                           .get_depth_scale();
  auto depth_stream = selection.get_stream(rs2_stream::RS2_STREAM_DEPTH)
                          .as<rs2::video_stream_profile>();

  auto i = depth_stream.get_intrinsics();
  intrinsics = (rs2_intrinsics*) alloc.alloc(sizeof(i));
  *intrinsics = i;
  rs2_fov(&i, fov);
  config.fov[0] = (fov[0] * M_PI)/180.0f;
  config.fov[1] = (fov[1] * M_PI)/180.0f;

  int index = 0;
  for (rs2::sensor sensor : selection.get_device().query_sensors()) {
    if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
      ++index;
      if (index == 1) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for
        // depth information
      }
      if (index == 2) {
        // RGB camera
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);
      }

      // if (index == 3) {
      //   sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
      // }
    }
  }

  return handle;
}


auto destroyHandle(struct RealsenseHandle* handle) -> Error {
  if (not handle) return Error::InvalidHandle;
  handle->pipe.~pipeline();
  handle->frame_q.~frame_queue();
  handle->pc.~pointcloud();
  // delete &(handle->frame_q);
  // delete &(handle->pc);
  return Error::NoError;
}


getPointFromPixel(RealsenseHandle* handle, float x, float y, double depth)
  -> Eigen::Vector3d {
  float points[3];
  float pixels[2] = {float(x),float(y)};
  rs2_deproject_pixel_to_point(
    points,
    intrinsics,
    pixels,
    depth);
  Eigen::Vector3d point;
  point.x() = points[0];
  point.y() = points[1];
  point.z() = points[2];
  return point;
}
auto getColorDepthPair(RealsenseHandle* handle, rs2::frameset& fs) {
  auto aligned_frames = handle->align.process(fs);
  rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
  rs2::video_frame depth_frame = aligned_frames.get_depth_frame();
  auto color_cv =
      cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)(color_frame.get_data()),
              cv::Mat::AUTO_STEP);
  cv::cvtColor(color_cv, color_cv, cv::COLOR_BGR2RGB);
  auto depth_cv =
      cv::Mat(cv::Size(640, 480), CV_16U, (void *)(depth_frame.get_data()),
              cv::Mat::AUTO_STEP);
  return std::make_pair(color_cv, depth_cv);
}
auto getPoints(RealsenseHandle* handle) -> std::variant<Error, rs2::points> {
  if (not handle) return Error::InvalidHandle;
  std::variant<Error, rs2::points> result(Error::NoFrameset);
  rs2::frame frame = handle->frame_q.wait_for_frame();
  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    rs2::video_frame depth_frame = fs.get_depth_frame();
    result.emplace<1>(handle->pc.calculate(depth_frame));
  } 
  return result;
}
auto getFrames(RealsenseHandle *handle)
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
auto resetLocalization(RealsenseHandle* handle) -> void {
  handle->slam.request_reset();
}
auto localizationMappingEnabled(RealsenseHandle* handle) -> bool {
  return handle->slam.mapping_module_is_enabled();
}
auto localizationLoopAdjustmentRunning(RealsenseHandle* handle) -> bool {
  return handle->slam.loop_BA_is_running();
}
auto runLocalization(RealsenseHandle *handle, const void *rerun_rec_ptr)
    -> std::variant<Error, Eigen::Matrix<double, 4, 4>> {
  std::variant<Error, Eigen::Matrix<double, 4, 4>> result;
  if (not handle) return Error::InvalidHandle;
  auto rec = (const rerun::RecordingStream*) rerun_rec_ptr;
  // if (handle->slam.terminate_is_requested()) return Error::
  while (handle->slam.loop_BA_is_running() or
         (not handle->slam.mapping_module_is_enabled())) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  rs2::frame frame = handle->frame_q.wait_for_frame();

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    auto [color_cv, depth_cv] = getColorDepthPair(handle, fs);
    auto timestamp_cv = fs.get_timestamp();

    handle->slam.feed_RGBD_frame(color_cv, depth_cv, timestamp_cv);
    auto pose = handle->slam.get_map_publisher()->get_current_cam_pose();
    Eigen::Matrix<double, 4,4> inverse_pose = pose.inverse();
    auto frame = handle->slam.get_frame_publisher()->draw_frame();

    std::string state =
        handle->slam.get_frame_publisher()->get_tracking_state().c_str();
    // printf("%s\n", state.c_str());
    if constexpr (LOG_TO_RERUN) {
      std::vector<std::shared_ptr<stella_vslam::data::landmark>> landmarks;
      std::set<std::shared_ptr<stella_vslam::data::landmark>> local_landmarks;
      std::vector<Eigen::Vector3f> points;
      std::vector<rerun::Color> colors;

      handle->slam.get_map_publisher()->get_landmarks(landmarks, local_landmarks);
      for (const auto& lm : landmarks) {
        if (not lm or lm->will_be_erased()) continue;
        points.emplace_back(lm->get_pos_in_world().cast<float>());
        const double score = lm->get_observed_ratio();
        turbo::Color3b color;
        turbo::GetColor(score, color);
        colors.emplace_back(color.r, color.g, color.b);
      }
      std::vector<rerun::Radius> radii(points.size(),0.005);
      for (const auto& lm : local_landmarks) {
        if (not lm or lm->will_be_erased()) continue;
        points.emplace_back(lm->get_pos_in_world().cast<float>());
        const double score = 0;
        turbo::Color3b color;
        turbo::GetColor(score, color);
        colors.emplace_back(color.r, color.g, color.b);
        radii.emplace_back(0.005);
      }
      rec->log("world/slam/landmarks",
               rerun::Points3D(points).with_colors(colors).with_radii(radii));
      if (state == "Tracking") {
        Eigen::Vector3f translations =
            inverse_pose.block<3, 1>(0, 3).cast<float>();
        Eigen::Matrix3f rotations =
            inverse_pose.block<3, 3>(0, 0).cast<float>();
        rec->log("world/slam/camera", rerun::Pinhole::from_fov_and_aspect_ratio(
                                    0.91524386, 640.0 / 480.0));
        rec->log("world/slam/camera",
                 rerun::Transform3D(rerun::Vec3D(translations.data()),
                                    rerun::Mat3x3(rotations.data())));
      }
      // rec->log("annotated",
      //          rerun::Image::from_rgb24(frame, {frame.cols, frame.rows}));
    }
    if (state == "Tracking") {
      auto translations = inverse_pose.col(3);
      auto rotations = inverse_pose.block<3, 3>(0, 0);
      // fmt::println("{}", *pose);
      // printf("%.3f %.3f %.3f\n", translations.x(), translations.y(),
      //        translations.z());
    }
    result.emplace<1>(inverse_pose);
  }
  return result;
}
}

int run_slam(){
  struct percep::Config config {
    .height = 640, .width = 480, .fps = 30, .enable_imu = true
  };

  mem::GeneralPurposeAllocator gpa;

  if (not rs_handle) {
    printf("Could not setup Realsense\n");
    return 1;
  }

  struct percep::RealsenseHandle* rs_ptr = percep::setupRealsense(gpa, config);

  printf("%" PRIu64 " is the size of RealsenseHandle. Alignment: %" PRIu64 "\n",
         percep::REALSENSE_HANDLE_SIZE, alignof(percep::RealsenseHandle));
  if (not rs_ptr) {
    printf("setupRealsense error\n");
    return 1;
  }
  while (true) {
    if (auto res = percep::runLocalization(rs_ptr, &rec);
        std::get_if<percep::Error>(&res)) {
      printf("getFrames error: %s\n", percep::getError(std::get<0>(res)));
    };
  }
}
