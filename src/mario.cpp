#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <format>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <rerun/recording_stream.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/types.h>
#include <taskflow/taskflow.hpp>
#include <thread>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "nav.hpp"
#include "pid.hpp"
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"

namespace po = boost::program_options;

/* zmq topic names */
const std::string topic_color = "color_frame";
const std::string topic_depth = "depth_frame";
const std::string topic_timestamp = "timestamp";
const std::string topic_pointcloud = "pointcloud";

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

  while (true) {
    std::vector<zmq::message_t> colorFrameMsg;
    std::vector<zmq::message_t> depthFrameMsg;
    std::vector<zmq::message_t> timestampMsg;

    zmq::recv_result_t result_color =
        zmq::recv_multipart(sub, std::back_inserter(colorFrameMsg));
    zmq::recv_result_t result_depth =
        zmq::recv_multipart(sub, std::back_inserter(depthFrameMsg));
    zmq::recv_result_t result_timestamp =
        zmq::recv_multipart(sub, std::back_inserter(timestampMsg));

    // check if recvd message is corrupt
    if (!result_color.has_value() || !result_depth.has_value() ||
        !result_timestamp.has_value()) {
      continue;
    }

    struct slam::rawColorDepthPair *frame_raw = new slam::rawColorDepthPair();

    frame_raw->colorFrame = colorFrameMsg[1].data();
    frame_raw->depthFrame = depthFrameMsg[1].data();
    frame_raw->timestamp = std::stod(timestampMsg[1].to_string());

    cv::Size colorFrameSize(realsense_config.color_i.height,
                            realsense_config.color_i.width);
    cv::Size depthFrameSize(realsense_config.depth_i.height,
                            realsense_config.depth_i.width);
    struct slam::RGBDFrame *frame_cv =
        slam::getColorDepthPair(frame_raw, colorFrameSize, depthFrameSize);

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

  while (true) {
    std::vector<zmq::message_t> pointcloud_msg;

    zmq::recv_result_t result_pointcloud =
        zmq::recv_multipart(sub, std::back_inserter(pointcloud_msg));

    // check if recvd message is corrupt
    if (!result_pointcloud.has_value())
      continue;

    std::vector<float> points_buffer;
    points_buffer.resize(pointcloud_msg[1].size() / sizeof(float));
    std::memcpy(points_buffer.data(), pointcloud_msg[1].data(),
                pointcloud_msg[1].size());
    int buffer_size = std::ceil(points_buffer.size() /
                                3); //  ceil -> buffer_size is not bigger
                                    // than cloud size (=buffer_size/3)
    // create pcl cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
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
      spdlog::error("Unable to fetch data from Pose Queue");
      continue;
    }

    /* transformation for pointcloud :-
    rotation to base link (FLU)
    translation to slam origin */
    Eigen::Affine3d T_pc;
    T_pc.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    T_pc.linear() = utils::T_camera_base.block<3, 3>(0, 0);

    // filter point cloud
    nav::preProcessPointCloud(nav_ctx, cloud, T_pc.matrix());

    // process grid map layer
    nav::processGridMapCells(nav_ctx, cloud);

    // log gridmap
    nav::log_gridmap(nav_ctx, rec);
  }
}

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::info);

  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(), "serial port to nucleo")(
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
  tf::Executor executor(2); // creating exectutor
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
  boost::asio::serial_port *nucleo;

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
  // open nucleo com
  nucleo = serial::open(io, SERIAL_PORT, BAUDRATE);
  if (not nucleo) {
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
  serial::close(nucleo);

  return 0;
}
