#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdint>
#include <format>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
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
  std::string topic_color = "color_frame";
  std::string topic_depth = "depth_frame";
  std::string topic_timestamp = "timestamp";
  std::string topic_pcl_header = "pcl_header";
  std::string topic_pointcloud = "point_cloud";

  /* realsense vars */
  struct utils::rs_config realsense_config{.height = 640,
                                           .width = 480,
                                           .fps = 30,
                                           .enable_imu = false};
  struct utils::rs_handler *rs_ptr;
  rs2::frame frame;

  /* slam vars */
  struct slam::slamHandle *slam_handler = new slam::slamHandle();
  Eigen::Matrix<double, 4, 4> current_pose;
  Eigen::Matrix<double, 4, 4> res;
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

  spdlog::info("Configuring Rover Peripherals");

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
  for (int i = 0; i < 100; i++) {
    // Wait for all configured streams to produce a frame
    frame = rs_ptr->frame_q.wait_for_frame();
  }

  /* CONFIGURING NUCLEO COM */
  // open nucleo com
  nucleo = serial::open(io, SERIAL_PORT, BAUDRATE);
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
    mapping_sub.set(zmq::sockopt::subscribe, topic_pcl_header);
  } catch (zmq::error_t &e) {
    spdlog::error(e.what());
  }

  /* CONFIGURING RERUN */
  rec.connect_grpc(rerun_url).exit_on_failure();

  // capture frame
  std::thread capture_frame([&]() {
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
	int64_t frame_nr = fs.get_frame_number();

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

	// log realsense pinhole view 
	
      }
    }
  });

  // slam
  std::thread slam([&]() {
    while (true) {
      std::vector<zmq::message_t> colorFrame_msg;
      std::vector<zmq::message_t> depthFrame_msg;
      std::vector<zmq::message_t> timestamp_msg;

      zmq::recv_result_t result_color =
          zmq::recv_multipart(slam_sub, std::back_inserter(colorFrame_msg));
      zmq::recv_result_t result_depth =
          zmq::recv_multipart(slam_sub, std::back_inserter(depthFrame_msg));
      zmq::recv_result_t result_timestamp =
          zmq::recv_multipart(slam_sub, std::back_inserter(timestamp_msg));

      // check if recvd message is corrupt
      if (!result_color.has_value() || !result_depth.has_value() ||
          !result_timestamp.has_value()) {
        continue;
      }

      struct slam::rawColorDepthPair *frame_raw = new slam::rawColorDepthPair();

      frame_raw->colorFrame = colorFrame_msg[1].data();
      frame_raw->depthFrame = depthFrame_msg[1].data();
      frame_raw->timestamp = std::stod(timestamp_msg[1].to_string());

      struct slam::RGBDFrame *frame_cv = slam::getColorDepthPair(frame_raw);

      res = slam::runLocalization(frame_cv, slam_handler);

      current_pose = utils::T_camera_base * res;
      auto translations = current_pose.col(3);
      auto rotations = current_pose.block<3, 3>(0, 0);

      // log to rerun
      std::string coordinates = std::format("x:{} y:{} yaw:{}", translations.x(),
                                            translations.y(), slam::yawfromPose(current_pose));
      rec.log("RoverPose", rerun::TextLog(coordinates));
      spdlog::debug(coordinates);

      // lock pose and write
      struct slam::slamPose pose = {.x = float(translations.x()),
                                    .y = float(translations.y()),
                                    .z = float(translations.z()),
                                    .yaw = slam::yawfromPose(current_pose)};
      // insert slam pose to queue
      poseQueue.produce(std::move(pose));
    }
  });

  // mapping
  std::thread mapping([&]() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      std::vector<zmq::message_t> pointcloud_msg;

      zmq::recv_result_t result_pointcloud =
          zmq::recv_multipart(mapping_sub, std::back_inserter(pointcloud_msg));

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
      struct slam::slamPose pose;
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

      // draw gridmap 
      nav::log_gridmap(nav_ctx, rec);
    }
  });

  // executes thread and task graph
  try {
    executor.run(taskflow).wait();
    capture_frame.join();
    slam.join();
    mapping.join();
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
