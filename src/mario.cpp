#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <ostream>
#include <vector>

#include <boost/program_options.hpp>

#include <librealsense2/rs.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <yasmin/blackboard/blackboard.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state_machine.hpp>

#include <taskflow/taskflow.hpp>

#include <mapping.hpp>
#include <serialib.h>
#include <yolo.hpp>

#include <mario.hpp>

namespace po = boost::program_options;

int main(int argc, char *argv[]) {

  // Command Line Options
  po::options_description desc("Allowed Options");

  desc.add_options()("help", "produce help message")(
      "serial", po::value<std::string>(),
      "serial port to nucleo")("cam", po::value<int>(), "port number of cam")(
      "model", po::value<std::string>(), "path to yolo model")(
      "labels", po::value<std::string>(), "path to label names");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  tf::Executor executor;  // creating exectutor
  tf::Taskflow taskflow;  // & taskflow graph obj
  rs2::pipeline pipe;     // creating rs
  rs2::config cfg;        // pipeline & cfg obj
  rs2::frameset frameset; // rs frames obj
  rs2::pointcloud pc;     // rs pointcloud
  rs2::points points;     // rs point obj
  Gridmap gridmap;        // store gridmap
  float grid_resolution = 0.001f;
  float height = 2.0;
  float prox_factor = 0.0;
  /* TEMP FOR IMU POSE IMPL  */
  rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
  rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};
  static auto last_time = std::chrono::high_resolution_clock::now();
  auto current_time = std::chrono::high_resolution_clock::now();
  float delta_time =
      std::chrono::duration<float>(current_time - last_time).count();
  Pose rover_pose;
  /* ---- */
  cv::VideoCapture capture(vm["cam"].as<int>(),
                           cv::CAP_V4L2); // cam obj for YOLO
  cv::Mat frame;                          // to store cam frame
  const std::string model_path =
      vm["model"].as<std::string>(); // YOLO model path
  const std::string labels_path =
      vm["labels"].as<std::string>(); // class label path
  YOLO8Detector detector(model_path, labels_path,
                         true);  // create onnx inference obj
  SafeQueue<cv::Mat> frameQueue; // queue to store raw frames
  SafeQueue<std::pair<int, cv::Mat>>
      processedQueue; // queue to store processed frames
  std::vector<std::string> class_names =
      utils::getClassNames(labels_path); // load class names
  serialib nucleo_com;                   // nucleo com obj
  std::string SERIAL_PORT =
      vm["serial"].as<std::string>(); // serial port to nucleo
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
            // calculating time difference
            last_time = std::chrono::high_resolution_clock::now();
            current_time = std::chrono::high_resolution_clock::now();
            delta_time =
                std::chrono::duration<float>(current_time - last_time).count();
            last_time = current_time;

            // Get accel data
            if (rs2::motion_frame accel_frame =
                    frameset.first_or_default(RS2_STREAM_ACCEL)) {
              accel_data = accel_frame.get_motion_data();
            } else {
              YASMIN_LOG_ERROR("Failed to retrieve accelerometer data\n");
            }

            // Get gyroscope data
            if (rs2::motion_frame gyro_frame =
                    frameset.first_or_default(RS2_STREAM_GYRO)) {
              gyro_data = gyro_frame.get_motion_data();
            } else {
              YASMIN_LOG_ERROR("Failed to retrieve gyroscope data\n");
            }

            // Update rover pose
            Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_data);
            Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_data);
            update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);
          })
          .name("slam");

  // mapping task
  auto mapping = taskflow
                     .emplace([&] {
                       // Process depth data to create point cloud
                       rs2::depth_frame depth_frame =
                           frameset.get_depth_frame();
                       rs2::points points = pc.calculate(depth_frame);

                       std::vector<Eigen::Vector3f> point_vectors;
                       for (size_t i = 0; i < points.size(); ++i) {
                         auto point = points.get_vertices()[i];
                         if (point.z) {
                           Eigen::Vector3f transformed_point =
                               rover_pose.orientation *
                                   Eigen::Vector3f(point.x, point.y, point.z) +
                               rover_pose.position;
                           point_vectors.push_back(transformed_point);
                         }
                       }

                       // Convert to PCL and filter
                       auto pcl_cloud = convert_to_pcl(point_vectors);
                       pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(
                           new pcl::PointCloud<pcl::PointXYZ>());
                       pcl::PassThrough<pcl::PointXYZ> passthrough;
                       passthrough.setInputCloud(pcl_cloud);
                       passthrough.setFilterFieldName("z");
                       passthrough.setFilterLimits(0.5, 5.0);
                       passthrough.filter(*passthrough_cloud);

                       pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
                           new pcl::PointCloud<pcl::PointXYZ>());
                       pcl::VoxelGrid<pcl::PointXYZ> voxel;
                       voxel.setInputCloud(passthrough_cloud);
                       voxel.setLeafSize(0.05f, 0.05f, 0.05f);
                       voxel.filter(*filtered_cloud);

                       point_vectors.clear();
                       for (const auto &point : filtered_cloud->points) {
                         point_vectors.emplace_back(point.x, point.y, point.z);
                       }

                       create_gridmap(gridmap, point_vectors, rover_pose,
                                      grid_resolution, height, prox_factor);
                     })
                     .name("mapping");
  auto path_planning = taskflow.emplace([&] {

		  }).name("path_planning");

  // state machine task
  //   try {
  //     std::string outcome = (*sm.get())(blackboard);
  //   } catch (const std::exception &e) {
  //     YASMIN_LOG_ERROR(e.what());
  //   }

  capture_frame.precede(yolo);
  capture_frame.precede(slam);
  slam.precede(mapping);

  while (true) {
    executor.run(taskflow).wait();
  }

  // release capture cam
  capture.release();

  return 0;
}
