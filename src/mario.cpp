#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstring>
#include <format>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <rerun.hpp>
#include <rerun/archetypes/text_log.hpp>
#include <spdlog/common.h>
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
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"

namespace po = boost::program_options;

// define blackboard keys
const std::string path_planning_flag = "plan_path";

class Navigate : public yasmin::State {

public:
  Navigate() : yasmin::State({"IDLE"}) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "IDLE";
  }
};

class Idle : public yasmin::State {

public:
  boost::asio::serial_port *nucleo;
  struct nav::navContext *nav_ctx;

  Idle(boost::asio::serial_port *serial, nav::navContext *ctx)
      : yasmin::State({"NAVIGATE", "IDLE"}), nucleo(serial), nav_ctx(ctx) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    // write stop command to nucleo
    tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(0.0, 0.0);
    std::string err =
        serial::get_error(serial::write_msg<struct tarzan::tarzan_msg>(
            nucleo, msg, tarzan::TARZAN_MSG_LEN));
    return "IDLE";
  }
};

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::info);

  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(), "serial port to nucleo")(
      "br", po::value<int>(),
      "baudrate for com with controller")("rerun_ip", po::value<std::string>(),
                                          "ip address of remote rerun viewer")(
      "gridmap_config", po::value<std::string>(), "gridmap parameters file");

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
  std::string topic_pcl_header = "pcl_header";
  std::string topic_pointcloud = "point_cloud";

  /* Yasmin vars */
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"IDLE"}); // state machine obj
  auto blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>(); // init blackboard

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
  utils::SafeQueue<struct slam::slamPose> poseQueue;

  /* path planning vars */
  struct nav::navContext *nav_ctx =
      nav::setupNav(vm["gridmap_config"].as<std::string>());
  const std::string elevation_layer = "elevation";
  const std::string cost_layer = "cost_map";

  /* serial com vars */
  std::string SERIAL_PORT = vm["serial"].as<std::string>();
  int BAUDRATE = vm["br"].as<int>();
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo;

  /* rerun vars */
  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");
  const std::string rerun_url =
      std::format("rerun+http://{}/proxy", vm["rerun_ip"].as<std::string>());

  rec.log("Sys Logs", rerun::TextLog("Configuring rover peripherals"));
  spdlog::info("Configuring Rover Peripherals");

  /* CONFIGURING REALSENSE */
  // init realsense handler
  rs_ptr = utils::setupRealsense(realsense_config);
  if (not rs_ptr) {
    spdlog::error("Unable to setup Realsense");
    return -1;
  }
  spdlog::info("Successfull setup of Realsense");
  // Successfull setup of realsense

  /* CONFIGURING NUCLEO COM */
  // open nucleo com
  nucleo = serial::open(io, SERIAL_PORT, BAUDRATE);
  spdlog::info(std::format("Succesfull connection to {}", SERIAL_PORT.c_str()));

  /* CONFIGURING STATE MACHINE */
  // Add states to the state machine
  sm->add_state("Navigate", std::make_shared<Navigate>(),
                {{"IDLE", "Idle"}});

  sm->add_state("Idle", std::make_shared<Idle>(nucleo, nav_ctx),
                {{"NAVIGATE", "Navigate"}, {"IDLE", "Idle"}});

  // set start of FSM as IDLE STATE
  sm->set_start_state("Idle");
  // set blacboard variables
  blackboard->set<bool>(path_planning_flag, false);

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

  /* TASK FLOW GRAPH */
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

              // get raw point cloud data and convert to pcl cloud
              rs2::points points = rs_ptr->pc.calculate(depthFrame);

              auto sp = points.get_profile().as<rs2::video_stream_profile>();

              // set pcl cloud header
              header.width = static_cast<uint32_t>(sp.width());
              header.height = static_cast<uint32_t>(sp.height());
              header.is_dense = false;

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
                memcpy(msg.data(), timestamp_string.data(),
                       timestamp_string.size());
                return msg;
              });
              if (!ret)
                spdlog::error("Error Publishing: topic_timestamp");
            }
          })
          .name("capture_frame");

  // slam task
  auto slam =
      taskflow
          .emplace([&]() {
            std::vector<zmq::message_t> colorFrame_msg;
            std::vector<zmq::message_t> depthFrame_msg;
            std::vector<zmq::message_t> timestamp_msg;

            zmq::recv_result_t result_color = zmq::recv_multipart(
                slam_sub, std::back_inserter(colorFrame_msg));
            zmq::recv_result_t result_depth = zmq::recv_multipart(
                slam_sub, std::back_inserter(depthFrame_msg));
            zmq::recv_result_t result_timestamp = zmq::recv_multipart(
                slam_sub, std::back_inserter(timestamp_msg));

            // check if recvd message is corrupt
            if (!result_color.has_value() || !result_depth.has_value() ||
                !result_timestamp.has_value())
              return;

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

            current_pose = utils::T_camera_to_ned * res;
            auto translations = current_pose.col(3);
            auto rotations = current_pose.block<3, 3>(0, 0);

            // log to rerun
            std::string coordinates =
                std::format("x:{} y:{} z:{}", translations.x(),
                            translations.y(), translations.z());
            rec.log("RoverPose", rerun::TextLog(coordinates));

            // lock pose and write
            struct slam::slamPose pose = {.x = float(translations.x()),
                                          .y = float(translations.y()),
                                          .z = float(translations.z()),
                                          .yaw =
                                              slam::yawfromPose(current_pose)};
            // insert slam pose to queue
            poseQueue.produce(std::move(pose));
          })
          .name("slam");

  // mapping task
  auto mapping =
      taskflow
          .emplace([&]() {
            std::vector<zmq::message_t> pointcloud_msg;
            std::vector<zmq::message_t> pcl_header;

            zmq::recv_result_t result_pointcloud = zmq::recv_multipart(
                mapping_sub, std::back_inserter(pointcloud_msg));

            // check if recvd message is corrupt
            if (!result_pointcloud.has_value()) 
              return;


            std::vector<float> points_buffer;
            points_buffer.resize(pointcloud_msg[1].size() / sizeof(float));
            std::memcpy(points_buffer.data(), pointcloud_msg[1].data(),
                        pointcloud_msg[1].size());

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            for (auto i = 0; i < points_buffer.size(); i += 3) {
              cloud->points[i].x = points_buffer[i + 2];  // forward
              cloud->points[i].y = -points_buffer[i];     // left-right
              cloud->points[i].z = -points_buffer[i + 1]; // up-down inversion
            }

            struct slam::slamPose pose;

            if (!poseQueue.consume(pose)) {
              spdlog::error("Unable to fetch data from Pose Queue");
            }

            float yaw_cos = std::cos(pose.yaw);
            float yaw_sin = std::sin(pose.yaw);
            // camera to ned transformation matrix
            const Eigen::Matrix<double, 4, 4> T_camera_to_map{
                {yaw_sin, 0, yaw_cos, pose.x},
                {-yaw_cos, 0, yaw_sin, pose.y},
                {0, -1, 0, pose.z},
                {0, 0, 0, 1}};

            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
            // transform pcl cloud
            pcl::transformPointCloud(*cloud, *transformed_cloud,
                                     T_camera_to_map);

          })
          .name("mapping");

  // state machine task
  //   try {
  //     std::string outcome = (*sm.get())(blackboard);
  //   } catch (const std::exception &e) {
  //     YASMIN_LOG_ERROR(e.what());
  //   }

  // define task graph
  capture_frame.precede(slam);
  capture_frame.precede(mapping);

  while (true) {
    try {
      executor.run(taskflow).wait();
    } catch (const std::exception &e) {
      std::cout << "Exception: {}" << e.what();
    }
  }

  // killing objects
  delete slam_handler;
  utils::destroyHandle(rs_ptr);
  serial::close(nucleo);

  return 0;
}
