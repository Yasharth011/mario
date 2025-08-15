#include <iostream>
#include <memory>
#include <ostream>
#include <vector>
#include <format>

#include <boost/program_options.hpp> 

#include <librealsense2/rs.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <yasmin/logs.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin/blackboard/blackboard.hpp>

#include <taskflow/taskflow.hpp> 

#include <serialib.h>
#include <yolo.hpp>

#include <mario.hpp>

namespace po = boost::program_options;


int main(int argc, char *argv[]) {

  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()
	  ("help", "produce help message")
	  ("serial", po::value<std::string>(), "serial port to nucleo")
	  ("cam", po::value<int>(), "port number of cam")
	  ("model", po::value<std::string>(), "path to yolo model")
	  ("labels", po::value<std::string>(), "path to label names")
	  ;

  po::variables_map vm; 
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  tf::Executor executor; // creating exectutor  
  tf::Taskflow taskflow; // & taskflow graph obj
  rs2::pipeline pipe; // creating rs
  rs2::config cfg;   // pipeline & cfg obj 
  rs2::pointcloud pc; // rs pointcloud 
  rs2::points points; // rs point obj
  cv::VideoCapture 
	  capture(vm["cam"].as<int>(), cv::CAP_V4L2); // cam obj for YOLO 
  cv::Mat frame; // to store cam frame
  const std::string model_path =
	  vm["model"].as<std::string>(); // YOLO model path 
  const std::string labels_path =
	  vm["labels"].as<std::string>(); // class label path 
  YOLO8Detector detector(model_path, labels_path, true); // create onnx inference obj 
  SafeQueue<cv::Mat> frameQueue; // queue to store raw frames
  SafeQueue<std::pair<int, cv::Mat>> processedQueue; // queue to store processed frames
  std::vector<std::string> class_names =
	  utils::getClassNames(labels_path); // load class names 
  serialib nucleo_com; // nucleo com obj 
  std::string SERIAL_PORT =
	  vm["serial"].as<std::string>(); // serial port to nucleo 
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"CONE_DETECTED"}); // state machine obj 
  auto blackboard = 
      std::make_shared<yasmin::blackboard::Blackboard>(); // init blackboard 

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
  
  // open serial device
  char errorOpening = nucleo_com.openDevice(SERIAL_PORT.c_str(), 9600);

  // check for errors
  if (errorOpening != 1) {
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
  auto capture_frame = taskflow.emplace([&](){
	capture.read(frame);
  }).name("capture_frame");

  // yolo task 
  auto yolo = taskflow.emplace([&](){
	std::vector<Detection> results = 
				detector.detect(frame);
	detector.drawBoundingBox(frame, results);
	cv::imshow("frame", frame); 
	cv::waitKey(1);
  }).name("yolo"); 

  // state machine task 
  //   try {
  //     std::string outcome = (*sm.get())(blackboard);
  //   } catch (const std::exception &e) {
  //     YASMIN_LOG_ERROR(e.what());
  //   }

  capture_frame.precede(yolo);
  while(true){
  executor.run(taskflow).wait();}
  
  // release capture cam
  capture.release();

  return 0;
}
