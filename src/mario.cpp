#include <boost/asio.hpp>
#include <boost/program_options.hpp>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <yasmin/blackboard/blackboard.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>

#include <taskflow/taskflow.hpp>

#include <zmq.hpp> 

#include <rerun.hpp>

#include <librealsense2/rs.hpp>

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
  const Eigen::Matrix<double, 4, 4> camera_to_ned_transform{
      {0, 0, 1, 0}, {-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 0, 1}};
  Eigen::Matrix<double, 4, 4> current_pose;
  Eigen::Matrix<double, 4, 4> res;

  /* path planning vars */

  /* Tarzan com vars */
  const std::string SERIAL_PORT =
      vm["serial"].as<std::string>(); // serial port to nucleo
  const int BAUDRATE = vm["br"].as<int>();
  boost::asio::io_context io;
  boost::asio::serial_port *nucleo;

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

  // connecting subscriber
  try {
    slam_sub.connect("inproc://realsense");
  } catch (zmq::error_t &e) {
    // e.what();
  }

  try {
    mapping_sub.connect("inproc://realsense");
  } catch (zmq::error_t &e) {
    // e.what();
  }

  /* TASK FLOW GRAPH */

  // capture task
  auto capture_frame = taskflow
                           .emplace([&]() {

                             // fetch frames from realsense
                             frame = rs_ptr->frame_q.wait_for_frame();

                             if (rs2::frameset fs = frame.as<rs2::frameset>()) {
                               struct slam::RGBDFrame *frame_cv =
                                   slam::getColorDepthPair(rs_ptr, fs);
                             }

                           })
                           .name("capture_frame");

  // slam task
  auto slam = taskflow.emplace([&]() {}).name("slam");

  // mapping task
  auto mapping = taskflow.emplace([&]() {}).name("mapping");

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

  executor.run(taskflow).wait();

  // killing objects
  delete slam_handler;
  utils::destroyHandle(rs_ptr);
  tarzan::close(nucleo);

  return 0;
}
