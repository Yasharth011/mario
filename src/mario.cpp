#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstring>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/types.h>
#include <taskflow/taskflow.hpp>
#include <thread>
#include <yasmin/blackboard/blackboard.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "nav.hpp"
#include "slam.hpp"
#include "tarzan.hpp"
#include "utils.hpp"

namespace po = boost::program_options;

class Navigate : public yasmin::State {

public:
  boost::asio::serial_port *nucleo;

  Navigate(boost::asio::serial_port *serial)
      : yasmin::State({"IDLE"}), nucleo(serial) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    return "IDLE";
  }
};

class Idle : public yasmin::State {

public:
  boost::asio::serial_port *nucleo;

  Idle(boost::asio::serial_port *serial)
      : yasmin::State({"NAVIGATE"}), nucleo(serial) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    return "NAVIGATE";
  }
};

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(), "serial port to nucleo")(
      "br", po::value<int>(), "baudrate for com with controller");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  /* taskflow vars */
  tf::Executor executor; // creating exectutor
  tf::Taskflow taskflow; // & taskflow graph obj

  /* zmq vars */
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, ZMQ_PUB);
  zmq::socket_t slam_sub(ctx, ZMQ_SUB);
  zmq::socket_t mapping_sub(ctx, ZMQ_SUB);
  std::string topic_color = "color_frame";
  std::string topic_depth = "depth_frame";
  std::string topic_timestamp = "timestamp";
  std::string topic_pointcloud = "point_cloud";

  /* Yasmin vars */
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"IDLE"}); // state machine obj
  auto blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>(); // init blackboard
  // set blacboard variables
  // blackboard->set<float>("right_arrow", 0.0);
  // blackboard->set<float>("left_arrow", 0.0);
  // blackboard->set<float>("cone", 0.0);

  /* realsense vars */
  struct utils::rs_config realsense_config{.height = 640,
                                           .width = 480,
                                           .fov = {0, 0},
                                           .depth_scale = 0.0,
                                           .fps = 30,
                                           .enable_imu = false};
  struct utils::rs_handler *rs_ptr;
  rs2::frame frame;

  /* slam vars */
  struct slam::slamHandle *slam_handler = new slam::slamHandle();
  Eigen::Matrix<double, 4, 4> current_pose;
  Eigen::Matrix<double, 4, 4> res;

  /* path planning vars */
  struct nav::navContext *nav_ctx = new nav::navContext();
  struct mapping::Slam_Pose pose;

  /* serial com vars */
  const std::string SERIAL_PORT = vm["seiral"].as<std::string>();
  const int BAUDRATE = vm["br"].as<int>();
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo;

  /* rerun vars */
  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");

  YASMIN_LOG_INFO("Configuring rover peripherals...");

  /* CONFIGURING REALSENSE */

  // init realsense handler
  rs_ptr = utils::setupRealsense(realsense_config);
  if (not rs_ptr) {
    // Could not setup Realsense
    return -1;
  }
  // Successfull setup of realsense

  /* CONFIGURING NUCLEO COM */

  // open nucleo com
  nucleo = tarzan::open(io, SERIAL_PORT, BAUDRATE);
  YASMIN_LOG_INFO("Succesfull connection to %s\n", SERIAL_PORT.c_str());

  /* CONFIGURING STATE MACHINE */

  // Add states to the state machine
  sm->add_state("Navigate", std::make_shared<Navigate>(nucleo),
                {{"IDLE", "Idle"}});

  sm->add_state("Idle", std::make_shared<Idle>(nucleo),
                {{"NAVIGATE", "Navigate"}});

  // set start of FSM as IDLE STATE
  sm->set_start_state("Idle");

  /* CONFIGURING ZMQ SOCKETS */

  // binding publisher
  try {
    pub.bind("inproc://realsense");
  } catch (zmq::error_t &e) {
    // e.what();
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
    // e.what();
  }

  try {
    mapping_sub.connect("inproc://realsense");

    mapping_sub.set(zmq::sockopt::subscribe, topic_pointcloud);
  } catch (zmq::error_t &e) {
    // e.what();
  }

  /* TASK FLOW GRAPH */

  // capture task
  auto capture_frame =
      taskflow
          .emplace([&]() {
            int ret; // error ret var

            // fetch frames from realsense
            frame = rs_ptr->frame_q.wait_for_frame();

            if (rs2::frameset fs = frame.as<rs2::frameset>()) {
              auto aligned_frames = rs_ptr->align.process(fs);

              rs2::video_frame colorFrame =
                  aligned_frames.first(RS2_STREAM_COLOR);
              rs2::video_frame depthFrame = aligned_frames.get_depth_frame();

              // get raw frame data
              const void *raw_colorFrame = colorFrame.get_data();
              const void *raw_depthFrame = depthFrame.get_data();

              // get length of frames
              size_t colorFrame_len = colorFrame.get_data_size();
              size_t depthFrame_len = depthFrame.get_data_size();

              double timestamp = fs.get_timestamp();

              // get raw point cloud data
              rs2::points points = rs_ptr->pc.calculate(depthFrame);
              std::vector<Eigen::Vector3f> raw_points;
              raw_points.reserve(points.size());
              const rs2::vertex *vertices = points.get_vertices();
              for (size_t i = 0; i < points.size(); i++) {
                if (vertices[i].z) {
                  Eigen::Matrix<double, 4, 1> vertices_vector{
                      {vertices[i].x}, {vertices[i].y}, {vertices[i].z}, {1.0}};

                  Eigen::Matrix<double, 4, 1> vertices_in_ned =
                      utils::camera_to_ned_transform * vertices_vector;

                  raw_points.push_back(vertices_in_ned.head<3>().cast<float>());
                }
              }

              // publishing messages
              ret = utils::publish_msg(
                  pub, topic_color, [raw_colorFrame, colorFrame_len]() {
                    zmq::message_t msg(colorFrame_len);
                    memcpy(msg.data(), raw_colorFrame, colorFrame_len);
                    return msg;
                  });
              if (!ret) { // error publishing
              }

              ret = utils::publish_msg(
                  pub, topic_depth, [raw_depthFrame, depthFrame_len]() {
                    zmq::message_t msg(depthFrame_len);
                    memcpy(msg.data(), raw_depthFrame, depthFrame_len);
                    return msg;
                  });
              if (!ret) { // error publishing
              }

              ret = utils::publish_msg(pub, topic_timestamp, [timestamp]() {
                std::string timestamp_string = std::to_string(timestamp);
                zmq::message_t msg(timestamp_string.size());
                memcpy(msg.data(), timestamp_string.data(),
                       timestamp_string.size());
                return msg;
              });
              if (!ret) { // error publishing
              }

              ret = utils::publish_msg(pub, topic_pointcloud, [raw_points]() {
                zmq::message_t msg(raw_points.size());
                memcpy(msg.data(), raw_points.data(), raw_points.size());
                return msg;
              });
              if (!ret) { // error publishing
              }
            }
          })
          .name("capture_frame");

  // slam task
  auto slam = taskflow
                  .emplace([&]() {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    std::vector<zmq::message_t> colorFrame_msg;
                    std::vector<zmq::message_t> depthFrame_msg;
                    std::vector<zmq::message_t> timestamp_msg;

                    zmq::recv_result_t result_color = zmq::recv_multipart(
                        slam_sub, std::back_inserter(colorFrame_msg));
                    zmq::recv_result_t result_depth = zmq::recv_multipart(
                        slam_sub, std::back_inserter(depthFrame_msg));
                    zmq::recv_result_t result_timestamp = zmq::recv_multipart(
                        slam_sub, std::back_inserter(timestamp_msg));

                    struct slam::rawColorDepthPair *frame_raw =
                        new slam::rawColorDepthPair();

                    frame_raw->colorFrame = colorFrame_msg[1].data();
                    frame_raw->depthFrame = depthFrame_msg[1].data();

                    std::string timestamp_string;
                    memcpy(timestamp_string.data(), timestamp_msg[1].data(),
                           sizeof(timestamp_msg[1].data()));
                    double timestamp = std::stod(timestamp_string);
                    frame_raw->timestamp = timestamp;

                    struct slam::RGBDFrame *frame_cv =
                        slam::getColorDepthPair(frame_raw);

                    res = slam::runLocalization(frame_cv, slam_handler, &rec);

                    current_pose = utils::camera_to_ned_transform * res;
                    auto translations = current_pose.col(3);
                    auto rotations = current_pose.block<3, 3>(0, 0);
                    spdlog::info("{} {} {}", translations.x(), translations.y(),
                                 translations.z());
                  })
                  .name("slam");

  // mapping task
  auto mapping =
      taskflow
          .emplace([&]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::vector<zmq::message_t> pointcloud_msg;

            zmq::recv_result_t result_pointcloud = zmq::recv_multipart(
                mapping_sub, std::back_inserter(pointcloud_msg));

            const float *points_buffer =
                static_cast<const float *>(pointcloud_msg[1].data());
            size_t points_buffer_size = pointcloud_msg[1].size();

            std::vector<Eigen::Vector3f> raw_points;

            for (size_t i = 0; i < points_buffer_size; i++) {
              raw_points.push_back(Eigen::Vector3f(points_buffer[i * 3 + 0],
                                                   points_buffer[i * 3 + 1],
                                                   points_buffer[i * 3 + 2]));
            }

            std::vector<Eigen::Vector3f> filtered_points =
                nav::processPointCloud(raw_points);

            nav::processPointCloud(raw_points);

            nav::updateMaps(nav_ctx, pose, filtered_points);
          })
          .name("mapping");

  // state machine task
  //   try {
  //     std::string outcome = (*sm.get())(blackboard);
  //   } catch (const std::exception &e) {
  //     YASMIN_LOG_ERROR(e.what());
  //   }

  // define task graph
  // capture_frame.precede(slam);
  // slam.precede(mapping);
  // mapping.precede(capture_frame);
  // mapping.precede(path_planning);

  while (true) {
    executor.run(taskflow).wait();
  }

  // killing objects
  delete slam_handler;
  utils::destroyHandle(rs_ptr);
  tarzan::close(nucleo);

  return 0;
}
