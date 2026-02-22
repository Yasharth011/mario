#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <format>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <memory>
#include <mutex>
#include <ompl/base/Path.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <rerun/recording_stream.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/types.h>
#include <taskflow/taskflow.hpp>
#include <thread>
#include <tuple>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "nav.hpp"
#include "pid.hpp"
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"

namespace po = boost::program_options;

#define EARTH_RADIUS 6378137.0; // earth radius in meters

/* zmq topic names */
const std::string topic_color = "color_frame";
const std::string topic_depth = "depth_frame";
const std::string topic_timestamp = "timestamp";
const std::string topic_pointcloud = "pointcloud";

struct mapReadySignal {
  std::mutex mtx;
  std::condition_variable cv;
  bool flag = false;
} map_sync;

/* function to capture & publish realsense frames */
void capture_frame(struct utils::rs_handler *rs_ptr, zmq::socket_t &pub) {

  rs2::frame frame;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int ret; // error ret var

    // fetch frames from realsense
    frame = rs_ptr->frame_q.wait_for_frame();

    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      auto aligned_frames = rs_ptr->align.process(fs);

      rs2::video_frame colorFrame = aligned_frames.first(RS2_STREAM_COLOR);
      rs2::video_frame depthFrame = aligned_frames.get_depth_frame();

      // get raw frame data
      const void *raw_colorFrame = colorFrame.get_data();
      const void *raw_depthFrame = depthFrame.get_data();

      // get length of frames
      size_t colorFrame_len = colorFrame.get_data_size();
      size_t depthFrame_len = depthFrame.get_data_size();

      double timestamp = fs.get_timestamp();

      // get raw point cloud data and convert to pcl cloud
      rs2::points points = rs_ptr->pc.calculate(depthFrame);
      // set point cloud vertices
      const rs2::vertex *vertices = points.get_vertices();
      std::vector<float> vertices_vector;
      for (size_t i = 0; i < points.size(); i++) {
        vertices_vector.push_back(vertices[i].x);
        vertices_vector.push_back(vertices[i].y);
        vertices_vector.push_back(vertices[i].z);
      }

      // publishing messages
      ret = utils::publish_msg(
          pub, topic_color, [raw_colorFrame, colorFrame_len]() {
            zmq::message_t msg(colorFrame_len);
            memcpy(msg.data(), raw_colorFrame, colorFrame_len);
            return msg;
          });
      if (!ret)
        spdlog::error("Error Publishing: topic_color");

      ret = utils::publish_msg(
          pub, topic_depth, [raw_depthFrame, depthFrame_len]() {
            zmq::message_t msg(depthFrame_len);
            memcpy(msg.data(), raw_depthFrame, depthFrame_len);
            return msg;
          });
      if (!ret)
        spdlog::error("Error Publishing: topic_depth");

      ret = utils::publish_msg(pub, topic_timestamp, [timestamp]() {
        std::string timestamp_string = std::to_string(timestamp);
        zmq::message_t msg(timestamp_string.size());
        memcpy(msg.data(), timestamp_string.data(), timestamp_string.size());
        return msg;
      });
      if (!ret)
        spdlog::error("Error Publishing: topic_timestamp");

      ret = utils::publish_msg(pub, topic_pointcloud, [vertices_vector]() {
        zmq::message_t msg(vertices_vector);
        return msg;
      });
      if (!ret)
        spdlog::error("Error Publishing: topic_pointcloud");
    }
  }
}

/* function to get slam pose */
void localize(struct slam::slamHandle *slam_handler,
              utils::SafeQueue<struct slam::slamPose> &poseQueue,
              zmq::socket_t &sub, const rerun::RecordingStream &rec,
              utils::rs_config realsense_config) {

  Eigen::Matrix<double, 4, 4> current_pose;
  Eigen::Matrix<double, 4, 4> res;
  std::vector<zmq::message_t> colorFrameMsg;
  std::vector<zmq::message_t> depthFrameMsg;
  std::vector<zmq::message_t> timestampMsg;
  zmq::recv_result_t result_color;
  zmq::recv_result_t result_depth;
  zmq::recv_result_t result_timestamp;
  struct slam::rawColorDepthPair *frame_raw = new slam::rawColorDepthPair;
  struct slam::RGBDFrame *frame_cv;
  struct slam::slamPose pose;

  while (true) {
    result_color = zmq::recv_multipart(sub, std::back_inserter(colorFrameMsg));
    result_depth = zmq::recv_multipart(sub, std::back_inserter(depthFrameMsg));
    result_timestamp =
        zmq::recv_multipart(sub, std::back_inserter(timestampMsg));

    // check if recvd message is corrupt
    if (!result_color.has_value() || !result_depth.has_value() ||
        !result_timestamp.has_value()) {
      continue;
    }

    frame_raw->colorFrame = colorFrameMsg[1].data();
    frame_raw->depthFrame = depthFrameMsg[1].data();
    frame_raw->timestamp = std::stod(timestampMsg[1].to_string());

    frame_cv = slam::getColorDepthPair(frame_raw);

    res = slam::runLocalization(frame_cv, slam_handler);

    current_pose = utils::T_camera_base * res;
    auto translations = current_pose.col(3);

    float x = translations.x();
    float y = translations.y();
    float z = translations.z();
    float yaw = slam::yawfromPose(current_pose);

    // insert slam pose to queue
    struct slam::slamPose pose = {.x = x, .y = y, .z = z, .yaw = yaw};
    poseQueue.produce(std::move(pose));

    // log pose
    std::string coordinates = std::format("x: {} y: {} yaw: {}", x, y, yaw);
    rec.log("SlamPose", rerun::TextLog(coordinates));

    // log realsense pinhole view
  }
}

/* function to create gridmap  */
void mapping(nav::navContext *nav_ctx,
             utils::SafeQueue<struct slam::slamPose> &poseQueue,
             zmq::socket_t &sub, const rerun::RecordingStream &rec,
             utils::rs_config realsense_config) {

  struct slam::slamPose pose;
  std::vector<zmq::message_t> pointcloud_msg;
  zmq::recv_result_t result_pointcloud;
  std::vector<float> points_buffer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Affine3d T_pc;

  while (!map_sync.flag) {

    result_pointcloud =
        zmq::recv_multipart(sub, std::back_inserter(pointcloud_msg));

    // check if recvd message is corrupt
    if (!result_pointcloud.has_value())
      continue;

    points_buffer.resize(pointcloud_msg[1].size() / sizeof(float));
    std::memcpy(points_buffer.data(), pointcloud_msg[1].data(),
                pointcloud_msg[1].size());
    int buffer_size = std::ceil(points_buffer.size() /
                                3); //  ceil -> buffer_size is not bigger
                                    // than cloud size (=buffer_size/3)
    // create pcl cloud
    cloud->width = buffer_size; // header.width;
    cloud->height = 1;          // header.height;
    cloud->is_dense = false;
    cloud->points.resize(buffer_size);

    for (auto i = 0; i < buffer_size; i += 3) {
      cloud->points[i].x = points_buffer[i];
      cloud->points[i].y = points_buffer[i + 1];
      cloud->points[i].z = points_buffer[i + 2];
    }

    if (!poseQueue.consume(pose)) {
      spdlog::error("GridMap : Unable to fetch data from Pose Queue");
      continue;
    }

    /* transformation for pointcloud :-
    rotation to base link (FLU)
    translation to slam origin */
    T_pc.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    T_pc.linear() = utils::T_camera_base.block<3, 3>(0, 0);

    // filter point cloud
    nav::preProcessPointCloud(nav_ctx, cloud, T_pc.matrix());

    // process grid map layer
    nav::processGridMapCells(nav_ctx, cloud);

    // log gridmap
    nav::log_gridmap(nav_ctx, rec);

    // notify map is ready
    std::lock_guard<std::mutex> lock(map_sync.mtx);
    map_sync.flag = true;
    map_sync.cv.notify_all();
  }
}

/* state to navigate to gps coordinate */
class StateMachine {
private:
  std::tuple<float, float> get_local_goal(struct tarzan::geodetic &current_gps,
                                          slam::slamPose &pose) {
    // difference in target and current gps coordiantes
    double dLat = DEG2RAD(target_gps.lat - current_gps.lat);
    double dLon = DEG2RAD(target_gps.lon - current_gps.lon);

    // Equirectangular approximation for distances
    double x_east = dLon * std::cos(DEG2RAD(current_gps.lat)) * EARTH_RADIUS;
    double y_north = dLat * EARTH_RADIUS;

    // calcaulte total distance
    double total_distance = std::sqrt((x_east * x_east) + (y_north * y_north));

    // calculating realtive angle of rover to target
    double target_angle_global = std::atan2(y_north, x_east);
    double rover_angle_global = DEG2RAD(90.0 - current_gps.head);
    double relative_angle =
        utils::normalize_angle(target_angle_global - rover_angle_global);

    double local_goal_dist =
        std::min(total_distance, (double)nav_ctx->params.grid_map_dim[1]);

    // claculating target (x, y) relative to rover
    double target_x = local_goal_dist * std::cos(relative_angle);
    double target_y = local_goal_dist * std::cos(relative_angle);

    // transformaing target to local goal
    float local_goal_x = pose.x + (target_x * std::cos(pose.yaw));
    float local_goal_y = pose.y + (target_y * std::cos(pose.yaw));

    return std::make_tuple(local_goal_x, local_goal_y);
  }

  void traverse_path(ob::PathPtr path) {
    auto geo_path =
        std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(path);
    if (!geo_path || geo_path->getStateCount() == 0) {
      spdlog::warn("State Machine: Path is empty or invalid");
      return;
    }
    const auto states = geo_path->getStates(); // get path states

    // record initial time for pid
    uint64_t previous = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    // traverse states
    int state_idx = 1;
    double linear_x, angular_z;
    while (state_idx < states.size()) {
      // get current pose
      slam::slamPose pose;
      if (!poseQueue.consume(pose)) {
        spdlog::error("State Machine : Unable to fetch data from Pose Queue");
        continue;
      }

      // claculate target distance
      auto state = states[state_idx]->as<ob::RealVectorStateSpace::StateType>();
      double target_x = state->values[0];
      double target_y = state->values[1];

      double dx = target_x - pose.x;
      double dy = target_y - pose.y;

      double distance = std::sqrt(dx * dx + dy * dy);

      // goal tolerance - distance to stop before goal
      if (distance < nav_ctx->params.grid_map_res) {
        state_idx++;
        continue;
      }

      // clalculate angular vel
      double target_yaw = std::atan2(dy, dx);
      double angular_error = utils::normalize_angle(target_yaw - pose.yaw);

      uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();

      if (std::abs(angular_error) < 0.01)
        angular_z = 0.0;
      else
        angular_z = std::clamp(
            control::computeCommand(pid_ctx, angular_error, now - previous),
            -5.5, 5.5);
      previous = now;

      // write drive cmd
      tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);
      serial::Error err = serial::write_msg<struct tarzan::tarzan_msg>(
          serial, msg, tarzan::TARZAN_MSG_LEN);
      if (err == serial::Error::WriteSuccess)
        spdlog::error("State Machine: writing drive cmd");
      else
        spdlog::error(std::format("State Machine: {}", serial::get_error(err)));
    }
  }

public:
  struct nav::navContext *nav_ctx;
  struct control::Pid *pid_ctx;
  boost::asio::serial_port *serial;
  utils::SafeQueue<struct slam::slamPose> &poseQueue;
  const struct tarzan::geodetic &target_gps;
  const struct tarzan::DiffDriveTwist &max_drive_cmd;

  StateMachine(struct nav::navContext *nav, struct control::Pid *pid,
               boost::asio::serial_port *ser,
               utils::SafeQueue<struct slam::slamPose> &queue,
               const struct tarzan::geodetic &target,
               const struct tarzan::DiffDriveTwist &cmd)
      : nav_ctx(nav), pid_ctx(pid), serial(ser), poseQueue(queue),
        target_gps(target), max_drive_cmd(cmd) {};

  int navGPS() {
    /* wait for map generation */
    std::unique_lock<std::mutex> lock(map_sync.mtx);
    map_sync.cv.wait(lock, [] { return map_sync.flag; });
    /* --- */

    // get current pose
    slam::slamPose pose;
    if (!poseQueue.consume(pose)) {
      spdlog::error("State Machine : Unable to fetch data from Pose Queue");
      return 0;
    }

    // get local goal coordinates
    struct tarzan::geodetic_msg geo_msg;
    serial::Error err = serial::read_msg<struct tarzan::geodetic_msg>(
        serial, &geo_msg, tarzan::GEODETIC_MSG_LEN);

    if (err == serial::AsioReadError || err == serial::CobsDecodeError) {
      spdlog::info(serial::get_error(err));
      return 0;
    }
    std::tuple goal_coords = get_local_goal(geo_msg.geo_data, pose);

    // set start and goal state
    ob::ScopedState<> start(nav_ctx->space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = pose.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = pose.y;

    ob::ScopedState<> goal(nav_ctx->space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] =
        get<0>(goal_coords);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] =
        get<1>(goal_coords);

    // get path to local goal
    ob::PathPtr path = nav::get_path(nav_ctx, start, goal);
  }
};

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::info);

  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(), "serial port to serial")(
      "br", po::value<int>(),
      "baudrate for com with controller")("rerun_ip", po::value<std::string>(),
                                          "ip address of remote rerun viewer")(
      "gridmap_config", po::value<std::string>(),
      "gridmap parameters file")("p", po::value<double>(), "proportional gain")(
      "i", po::value<double>(), "integral gain")("d", po::value<double>(),
                                                 "differential gain");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  /* taskflow vars */
  tf::Executor executor(1); // creating exectutor
  tf::Taskflow taskflow;    // & taskflow graph obj

  /* zmq vars */
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, ZMQ_PUB);
  zmq::socket_t slam_sub(ctx, ZMQ_SUB);
  zmq::socket_t mapping_sub(ctx, ZMQ_SUB);

  /* realsense vars */
  struct utils::rs_config realsense_config{
      .height = 640, .width = 480, .fps = 30, .enable_imu = false};
  struct utils::rs_handler *rs_ptr;

  /* slam vars */
  struct slam::slamHandle *slam_handler = new slam::slamHandle();
  utils::SafeQueue<struct slam::slamPose> poseQueue;

  /* path planning vars */
  struct nav::navContext *nav_ctx =
      nav::setupNav(vm["gridmap_config"].as<std::string>());

  /* serial com vars */
  std::string SERIAL_PORT = vm["serial"].as<std::string>();
  int BAUDRATE = vm["br"].as<int>();
  boost::asio::io_context io;
  boost::asio::serial_port *serial;

  /* rerun vars */
  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");
  const std::string rerun_url =
      std::format("rerun+http://{}/proxy", vm["rerun_ip"].as<std::string>());

  spdlog::info("Configuring Rover Peripherals...");

  /* pid vars */
  struct control::Pid *pid_ctx;
  pid_ctx = control::initPid(vm["p"].as<double>(), vm["i"].as<double>(),
                             vm["d"].as<double>());

  /* CONFIGURING REALSENSE */
  // init realsense handler
  rs_ptr = utils::setupRealsense(realsense_config);
  if (not rs_ptr) {
    spdlog::error("Unable to setup Realsense");
    return -1;
  }
  spdlog::info("Successfull setup of Realsense");
  // Camera warmup - dropping several first frames to let auto-exposure
  // stabilize
  rs2::frame frame;
  for (int i = 0; i < 100; i++) {
    // Wait for all configured streams to produce a frame
    frame = rs_ptr->frame_q.wait_for_frame();
  }

  /* CONFIGURING NUCLEO COM */
  // open serial com
  serial = serial::open(io, SERIAL_PORT, BAUDRATE);
  if (not serial) {
    spdlog::error("Unable to setup Nucleo serial port");
    return -1;
  }
  spdlog::info(std::format("Succesfull connection to {}", SERIAL_PORT.c_str()));

  /* CONFIGURING ZMQ SOCKETS */
  // binding publisher
  try {
    pub.bind("inproc://realsense");
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  // sleep for lazy subs
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // connecting subscriber
  try {
    slam_sub.connect("inproc://realsense");
    slam_sub.set(zmq::sockopt::subscribe, topic_color);
    slam_sub.set(zmq::sockopt::subscribe, topic_depth);
    slam_sub.set(zmq::sockopt::subscribe, topic_timestamp);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  try {
    mapping_sub.connect("inproc://realsense");
    mapping_sub.set(zmq::sockopt::subscribe, topic_pointcloud);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  /* CONFIGURING RERUN */
  rec.connect_grpc(rerun_url).exit_on_failure();

  /* CONFIGURING THREADS */
  std::thread capture_thread(capture_frame, rs_ptr, std::ref(pub));
  std::thread localize_thread(localize, slam_handler, std::ref(poseQueue),
                              std::ref(slam_sub), std::ref(rec),
                              realsense_config);
  std::thread mapping_thread(mapping, nav_ctx, std::ref(poseQueue),
                             std::ref(mapping_sub), std::ref(rec),
                             realsense_config);

  /* CONFIGURING TASK-GRAPH */

  /* EXECUTING THREAD & TASK-GRAPH */
  try {
    executor.run(taskflow).wait();
    capture_thread.join();
    localize_thread.join();
    mapping_thread.join();
  } catch (const std::exception &e) {
    spdlog::info(std::format("Exception: {}", e.what()));
  }

  // killing objects
  delete slam_handler;
  delete nav_ctx;
  delete pid_ctx;
  utils::destroyHandle(rs_ptr);
  serial::close(serial);

  return 0;
}
