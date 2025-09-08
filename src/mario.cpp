#include <chrono>
#include <cmath>
#include <iostream>
#include <librealsense2/h/rs_types.h>
#include <memory>
#include <ostream>
#include <rerun/recording_stream.hpp>
#include <vector>

#include <boost/program_options.hpp>

#include <librealsense2/rs.hpp>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include <motionplanner.h>
#include <mapping.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <yasmin/blackboard/blackboard.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>

#include <taskflow/taskflow.hpp>

#include <rerun.hpp>

#include <tarzan.hpp>
#include <yolo.hpp>

#include <mario.hpp>

#include <librealsense2/h/rs_types.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/rsutil.h>

#include "alloc.hpp"
#include "cppturbo.hpp"
#include "collection_adapters.hpp"
#include "localize.hpp"

#include <stella_vslam/data/landmark.h>
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
namespace po = boost::program_options;

class Navigate : public yasmin::State {

public:
  Tarzan &nucleo_com;

  Navigate(Tarzan &com)
      : yasmin::State({"IDLE", "ARROW_DETECTED", "CONE_DETECTED"}),
        nucleo_com(com) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    return "IDLE";
  }
};

class Idle : public yasmin::State {

public:
  Tarzan &nucleo_com;

  Idle(Tarzan &com)
      : yasmin::State({"NAVIGATE", "ARROW_DETECTED", "CONE_DETECTED"}),
        nucleo_com(com) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    return "NAVIGATE";
  }
};

class Cone_Detected : public yasmin::State {

public:
  Tarzan &nucleo_com;

  Cone_Detected(Tarzan &com) : yasmin::State({"IDLE"}), nucleo_com(com) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    return "IDLE";
  }
};

class Arrow_Detected : public yasmin::State {

public:
  Tarzan &nucleo_com;

  Arrow_Detected(Tarzan &com)
      : yasmin::State({"NAVIGATE", "IDLE"}), nucleo_com(com) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "Idle";
  }
};

int main(int argc, char *argv[]) {

  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(),
      "serial port to nucleo")("cam", po::value<int>(), "port number of cam")("br", po::value<int>(), "baudrate for com with controller")(
      "model", po::value<std::string>(), "path to yolo model")(
      "labels", po::value<std::string>(), "path to label names");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  /* taskflow vars*/
  tf::Executor executor; // creating exectutor
  tf::Taskflow taskflow; // & taskflow graph obj

struct RealsenseHandle {
  rs2::frame_queue frame_q;
  rs2::pointcloud pc;
  rs2::pipeline pipe;
  rs2::align align;
  rs2::config cfg;
  stella_vslam::system slam;
  stella_vslam::config slam_cfg;
};
// check struct in mario.hpp 

  /* rerun vars */
  const auto rec = rerun::RecordingStream("TEAM RUDRA AUTONOMOUS - mario");

  /* path planning vars */

  /* YOLO vars */
  cv::VideoCapture capture(vm["cam"].as<int>(),
                           cv::CAP_V4L2); // cam obj for YOLO
  cv::Mat frame;                          // to store cam frame
  const std::string model_path =
      vm["model"].as<std::string>(); // YOLO model path
  const std::string labels_path =
      vm["labels"].as<std::string>(); // class label path
  YOLO8Detector detector(model_path, labels_path,
                         true); // create onnx inference obj
  // SafeQueue<cv::Mat> frameQueue; // queue to store raw frames
  // SafeQueue<std::pair<int, cv::Mat>>
  // processedQueue; // queue to store processed frames
  std::vector<std::string> class_names =
      utils::getClassNames(labels_path); // load class names

  /* Tarzan com vars*/
  const std::string SERIAL_PORT =
      vm["serial"].as<std::string>(); // serial port to nucleo
  const int BAUDRATE = vm["br"].as<int>();
  boost::asio::io_context io;
  Tarzan nucleo_com(io, SERIAL_PORT, BAUDRATE);

  /* Yasmin vars*/
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"CONE_DETECTED"}); // state machine obj
  auto blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>(); // init blackboard
  // set blacboard variables
  blackboard->set<float>("right_arrow", 0.0);
  blackboard->set<float>("left_arrow", 0.0);
  blackboard->set<float>("cone", 0.0);

  /* CONFIGURING PERIPHERALS */

  YASMIN_LOG_INFO("Configuring rover peripherals...");

  /* CONFIGURING REALSENSE */

  // Enable Realsense Streams
  cfg.enable_stream(RS2_STREAM_DEPTH);
  cfg.enable_stream(RS2_STREAM_GYRO);
  cfg.enable_stream(RS2_STREAM_ACCEL);
  cfg.enable_stream(RS2_STREAM_COLOR);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2);

  // Starting realsense pipeline
  try {
    pipe.start(cfg);
    YASMIN_LOG_INFO("Succesful connection to RealSense");
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("RealSense : %s", e.what());
    return -1;
  }

  /* CONFIGURING NUCLEO COM */

  // check for errors
  if (!nucleo_com.open()) {
    YASMIN_LOG_ERROR("Nucleo : Error opening device");
    return -1;
  }
  YASMIN_LOG_INFO("Succesful connection to %s\n", SERIAL_PORT.c_str());

  /* CONFIGURING CAM */

  // check for errors
  if (!capture.isOpened()) {
    YASMIN_LOG_ERROR("Cam : Error opening device");
    return -1;
  }
  YASMIN_LOG_INFO("Succesful connection to cam\n");

  /* CONFIGURING STATE MACHINE */

  // Add states to the state machine
  sm->add_state("Navigate", std::make_shared<Navigate>(nucleo_com),
                {{"IDLE", "Idle"},
                 {"ARROW_DETECTED", "Arrow_Detected"},
                 {"CONE_DETECTED", "Cone_Detected"}});

  sm->add_state("Idle", std::make_shared<Idle>(nucleo_com),
                {{"NAVIGATE", "Navigate"},
                 {"ARROW_DETECTED", "Arrow_Detected"},
                 {"CONE_DETECTED", "Cone_Detected"}});

  sm->add_state("Arrow_Detected", std::make_shared<Arrow_Detected>(nucleo_com),
                {{"IDLE", "Idle"}, {"NAVIGATE", "Navigate"}});

  sm->add_state("Cone_Detected", std::make_shared<Cone_Detected>(nucleo_com),
                {{"IDLE", "Idle"}});

  // set start of FSM as IDLE STATE
  sm->set_start_state("Idle");

  /* TASK FLOW GRAPH */

  // capture task
  auto capture_frame =
      taskflow
          .emplace([&]() {
            capture.read(frame); // fetch cam frames

            // fetch frames from realsense
            try {
              frameset = pipe.wait_for_frames();
            } catch (const rs2::error &e) {
              YASMIN_LOG_ERROR("Realsense error : %s\n", e.what());
            }
          })
          .name("capture_frame");

  // yolo task
  auto yolo = taskflow
                  .emplace([&]() {
                    std::vector<Detection> results = detector.detect(frame);

                    detector.drawBoundingBox(frame, results);

                    for (Detection result : results) {
                      if (result.classId == 0)
                        blackboard->set<float>("right_arrow", result.conf);
                      else if (result.classId == 1)
                        blackboard->set<float>("left_arrow", result.conf);
                      else if (result.classId == 2)
                        blackboard->set<float>("cone", result.conf);
                      else
                        continue;
                    }

                    cv::imshow("frame", frame);
                    cv::waitKey(1);
                  })
                  .name("yolo");

  // slam task (using rs imu for rover's pose for now)
  auto slam =
      taskflow
          .emplace([&]() {
          })
          .name("slam");

  // mapping task
  auto mapping =
      taskflow
          .emplace([&]() {
          })
          .name("mapping");

  auto path_planning = taskflow.emplace([&](){

		  }).name("path_planning");

  // state machine task
  //   try {
  //     std::string outcome = (*sm.get())(blackboard);
  //   } catch (const std::exception &e) {
  //     YASMIN_LOG_ERROR(e.what());
  //   }

  // define task graph 
  capture_frame.precede(yolo);
  capture_frame.precede(slam);
  slam.precede(mapping);
  mapping.precede(capture_frame);
  mapping.precede(path_planning);

  executor.run(taskflow).wait();

  // release capture cam
  capture.release();

  return 0;
}
