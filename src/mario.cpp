#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <format>
#include <iterator>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/rs.hpp>
#include <mapping.h>
#include <mutex>
#include <opencv2/opencv.hpp>
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
#include "pid.hpp"
#include "serial.hpp"
#include "slam.hpp"
#include "utils.hpp"

namespace po = boost::program_options;

// define blackboard keys
const std::string path_planning_flag = "plan_path";

class Navigate : public yasmin::State {

private:
  struct mapping::Slam_Pose pose;

public:
  boost::asio::serial_port *nucleo;
  struct nav::navContext *nav_ctx;
  struct control::Pid *pid_ctx;
  utils::SafeQueue<mapping::Slam_Pose> &poseQueue;

  Navigate(boost::asio::serial_port *serial, nav::navContext *ctx,
           control::Pid *pid, utils::SafeQueue<mapping::Slam_Pose> &Queue)
      : yasmin::State({"IDLE"}), nucleo(serial), nav_ctx(ctx), pid_ctx(pid),
        poseQueue(Queue) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    if (nav_ctx->pruned_path.size() < 2)
      return "IDLE";
    planning::Node previous_start = nav_ctx->current_start;
    bool found_start = false;
    for (size_t i = 0; i < nav_ctx->pruned_path.size() - 1; ++i) {
      if (!found_start) {
        if (nav_ctx->pruned_path[i] == nav_ctx->current_start) {
          found_start = true;
        } else {
          continue; // keep skipping
        }
      }
      planning::Node local_start = nav_ctx->pruned_path[i];
      planning::Node local_goal = nav_ctx->pruned_path[i + 1];
      // Ensure we only start executing from where the rover currently is
      if (local_start.x != nav_ctx->current_start.x ||
          local_start.y != nav_ctx->current_start.y) {
        continue;
      }

      while (true) {
        if (!poseQueue.consume(pose)) {
          spdlog::error("State Machine : Unable to fetch data from Pose Queue");
        }
        float x = pose.x;
        float y = pose.y;
        float yaw = pose.yaw;
        // compute linear error
        double linear_error =
            sqrt(pow(local_goal.x * nav::grid_resolution - x, 2) +
                 pow(local_goal.y * nav::grid_resolution - y, 2));

        double angle_error = atan2(local_goal.y * nav::grid_resolution - y,
                                   local_goal.x * nav::grid_resolution - x) -
                             yaw;
        if (linear_error <= 0.5 && angle_error < 1.0)
          break;
        // compute time difference
        uint64_t current_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        uint64_t dt = current_time - pid_ctx->last_time;

        float linear_x = control::computeCommand(pid_ctx, linear_error, dt);
        float angular_z = control::computeCommand(pid_ctx, angle_error, dt);

        pid_ctx->last_time = current_time;
        // write tarzan message
        tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(linear_x, angular_z);
        std::string err =
            serial::get_error(serial::write_msg<struct tarzan::tarzan_msg>(
                nucleo, msg, tarzan::TARZAN_MSG_LEN));
        spdlog::info(err);
      }
      control::resetPid(pid_ctx);

      nav_ctx->current_start = local_goal;
      previous_start = nav_ctx->current_start;
      if (nav_ctx->current_start == nav_ctx->final_goal) {
        // goal reached
        blackboard->set<bool>(path_planning_flag, false);
        break; // Exit movement loop
      }
    }
    return "IDLE";
  }
};

class Explore : public yasmin::State {

private:
  struct mapping::Slam_Pose pose;

public:
  boost::asio::serial_port *nucleo;
  struct nav::navContext *nav_ctx;
  struct control::Pid *pid_ctx;
  utils::SafeQueue<mapping::Slam_Pose> &poseQueue;

  Explore(boost::asio::serial_port *serial, nav::navContext *ctx,
          control::Pid *pid, utils::SafeQueue<mapping::Slam_Pose> &Queue)
      : yasmin::State({"IDLE"}), nucleo(serial), nav_ctx(ctx), pid_ctx(pid),
        poseQueue(Queue) {};

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
      : yasmin::State({"NAVIGATE", "IDLE", "EXPLORE"}), nucleo(serial),
        nav_ctx(ctx) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    // write stop command to nucleo
    tarzan::tarzan_msg msg = tarzan::get_tarzan_msg(0.0, 0.0);
    std::string err =
        serial::get_error(serial::write_msg<struct tarzan::tarzan_msg>(
            nucleo, msg, tarzan::TARZAN_MSG_LEN));
    spdlog::info(err);

    if (blackboard->get<bool>(path_planning_flag)) {
      spdlog::info("inisde IDLE");
      if (!nav::findCurrentGoal(nav_ctx))
        spdlog::info("exploring");
      return "EXPLORE";

      std::vector<planning::Node> dense_path;
      if (nav::findPath(nav_ctx, dense_path)) {
        nav_ctx->pruned_path = nav::prunePath(dense_path);
        spdlog::info("navigating");
        return "NAVIGATE";
      }

      // if neither goal or path found create new map
      blackboard->set<bool>(path_planning_flag, false);
    }
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
      "proportional", po::value<double>(),
      "proportional gain")("integral", po::value<double>(), "integral gain")(
      "differential", po::value<double>(), "differential gain");

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

  /* path planning vars */
  struct nav::navContext *nav_ctx = new nav::navContext();
  utils::SafeQueue<struct mapping::Slam_Pose> poseQueue;
  std::mutex poseMtx;
  bool poseAvail = false;
  std::condition_variable poseCV;
  int counter = 0;
  int adder = 0;

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
  pid_ctx = control::initPid(vm["proportional"].as<double>(),
                             vm["integral"].as<double>(),
                             vm["differential"].as<double>());

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
  sm->add_state("Navigate",
                std::make_shared<Navigate>(nucleo, nav_ctx, pid_ctx, poseQueue),
                {{"IDLE", "Idle"}});

  sm->add_state("Explore",
                std::make_shared<Explore>(nucleo, nav_ctx, pid_ctx, poseQueue),
                {{"IDLE", "Idle"}});

  sm->add_state(
      "Idle", std::make_shared<Idle>(nucleo, nav_ctx),
      {{"NAVIGATE", "Navigate"}, {"EXPLORE", "Explore"}, {"IDLE", "Idle"}});

  // set start of FSM as IDLE STATE
  sm->set_start_state("Idle");
  // set blacboard variables
  blackboard->set<bool>(path_planning_flag, false);
  // redirect yasmin logger to spdlogger
  yasmin::set_loggers(utils::yasmin_to_spdlog);

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

              // get raw point cloud data
              rs2::points points = rs_ptr->pc.calculate(depthFrame);

              const rs2::vertex *vertices = points.get_vertices();

              std::vector<float> vertices_vector;
              vertices_vector.reserve(points.size());

              for (size_t i = 0; i < points.size(); i++) {

                if (vertices[i].z) {
                  vertices_vector.push_back(vertices[i].x);
                  vertices_vector.push_back(vertices[i].y);
                  vertices_vector.push_back(vertices[i].z);
                }
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

              ret = utils::publish_msg(pub, topic_pointcloud,
                                       [vertices_vector]() {
                                         zmq::message_t msg(vertices_vector);
                                         return msg;
                                       });
              if (!ret)
                spdlog::error("Error Publishing: topic_pointcloud");
            }
            return 0;
          })
          .name("capture_frame");

  // slam task
  auto slam = taskflow
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

                    // log to rerun
                    std::string coordinates =
                        std::format("x:{} y:{} z:{}", translations.x(),
                                    translations.y(), translations.z());
                    // spdlog::info(coordinates);
                    rec.log("RoverPose", rerun::TextLog(coordinates));
  
                    // lock pose and write
                    struct mapping::Slam_Pose pose = {
                        .x = float(translations.x()),
                        .y = float(translations.y()),
                        .z = float(translations.z()),
                        .yaw = slam::yawfromPose(current_pose)};
                    // insert slam pose to queue
                    poseQueue.produce(std::move(pose));
                    return 0;
                  })
                  .name("slam");

  // mapping task
  auto mapping =
      taskflow
          .emplace([&]() {
            // if path_planning flag do not map
            if (blackboard->get<bool>(path_planning_flag))
              return;

            std::vector<zmq::message_t> pointcloud_msg;

            zmq::recv_result_t result_pointcloud = zmq::recv_multipart(
                mapping_sub, std::back_inserter(pointcloud_msg));

            std::vector<float> points_buffer;
            points_buffer.resize(pointcloud_msg[1].size() / sizeof(float));
            std::memcpy(points_buffer.data(), pointcloud_msg[1].data(),
                        pointcloud_msg[1].size());

            struct mapping::Slam_Pose pose;

            if (!poseQueue.consume(pose)) {
              spdlog::error("Mapping: Unable to fetch data from Pose Queue");
            }

            Eigen::Matrix3f R; // rotation matrix
            R = Eigen::AngleAxisf(pose.yaw, Eigen::Vector3f::UnitZ());
            // translation matrix
            Eigen::Vector3f T(pose.x, pose.y, pose.z);

            std::vector<Eigen::Vector3f> transformed_points;

            for (auto i = 0; i < points_buffer.size(); i += 3) {
              float dx = points_buffer[i + 2];  // forward
              float dy = -points_buffer[i];     // left-right
              float dz = -points_buffer[i + 1]; // up-down inversion

              Eigen::Vector3f p_cam(dx, dy, dz);

              // Applying rotation and translation together
              Eigen::Vector3f p_world = R * p_cam + T;
              transformed_points.push_back(p_world);
            }

            std::vector<Eigen::Vector3f> filtered_points =
                nav::processPointCloud(transformed_points);

            nav::updateMaps(nav_ctx, pose, filtered_points, rec);
            counter = nav_ctx->gridmap.occupancy_grid.size() - adder;
            if (counter >= nav::map_limit) {
              spdlog::info("path planning flag set");
              blackboard->set<bool>(path_planning_flag, true);
              adder += nav::map_limit; // Increment adder so the condition
                                       // isn't met again immediately
            }
          })
          .name("mapping");

  // state machine task
  executor.silent_async([&]() {
    try {
      std::string outcome = (*sm.get())(blackboard);
      spdlog::info(std::format("YASMIN OUTCOME : {}", outcome));
    } catch (const std::exception &e) {
      spdlog::error(e.what());
    }
  });

  // define task graph
  capture_frame.precede(slam);
  slam.precede(mapping);

  while (true) {
    try {
      executor.run(taskflow).wait();
    } catch (const std::exception &e) {
      spdlog::info(std::format("Exception: {}", e.what()));
    }
  }

  // killing objects
  delete slam_handler;
  delete nav_ctx;
  delete pid_ctx;
  utils::destroyHandle(rs_ptr);
  serial::close(nucleo);

  return 0;
}
