#include <librealsense2/rs.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <vector>
#include <yasmin/logs.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>

#include <serialib.h>
#include <yolo.hpp>

#include <mario.hpp>

class Navigate : public yasmin::State {

public:
  serialib serial;

  Navigate(serialib x)
      : yasmin::State({"IDLE", "ARROW_DETECTED", "CONE_DETECTED"}),
        serial(x) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {

    return "IDLE";
  }
};

class Idle : public yasmin::State {

public:
  serialib serial;

  Idle(serialib x)
      : yasmin::State({"NAVIGATE", "ARROW_DETECTED", "CONE_DETECTED"}),
        serial(x) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "NAVIGATE";
  }
};

class Cone_Detected : public yasmin::State {

public:
  serialib serial;

  Cone_Detected(serialib x) : yasmin::State({"IDLE"}), serial(x) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "IDLE";
  }
};

class Arrow_Detected : public yasmin::State {

public:
  serialib serial;

  Arrow_Detected(serialib x)
      : yasmin::State({"NAVIGATE", "IDLE"}), serial(x) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "Idle";
  }
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("Configuring rover peripherals...");

  // Create Pipeline and configure realsense
  rs2::pipeline pipe;
  rs2::config cfg;

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

  rs2::pointcloud pc;
  rs2::points points;

  // serial object
  serialib nucleo_com;

  char *SERIAL_PORT = argv[1];

  // open serial device
  char errorOpening = nucleo_com.openDevice(SERIAL_PORT, 9600);

  // check for errors
  if (errorOpening != 1) {
    YASMIN_LOG_ERROR("Nucleo : Error opening device");
    return -1;
  }

  YASMIN_LOG_INFO("Succesful connection to %s\n", SERIAL_PORT);

  // open camera for object detection
  cv::VideoCapture capture(0, cv::CAP_V4L2); // default camera for now

  // check for errors
  if (!capture.isOpened()) {
    YASMIN_LOG_ERROR("Cam : Error opening device");
    return -1;
  }

  // load YOLOV8 model
  const std::string model_path = "./model/arrow_model.onnx";
  const std::string labels_path = "./model/labels.names";
  YOLO8Detector detector(model_path, labels_path, true);

  // Thread-safe queues for vedio processing
  SafeQueue<cv::Mat> frameQueue;
  SafeQueue<std::pair<int, cv::Mat>> processedQueue;

  // Threads for detection
  // Capture Thread
  std::thread capture_thread([&]() {
    cv::Mat frame;
    int frame_count = 0;
    while (capture.read(frame)) {
      frameQueue.enqueue(frame.clone());
      frame_count++;
    }
    frameQueue.setFinished();
  });

  // Processing Thread
  std::thread processing_thread([&]() {
    cv::Mat frame;
    int frameIndex = 0;
    while (frameQueue.dequeue(frame)) {
      std::vector<Detection> results = detector.detect(frame);
      detector.drawBoundingBox(frame, results);
      processedQueue.enqueue(std::make_pair(frameIndex++, frame));
    }
    processedQueue.setFinished();
  });

  // Writting Thread
  std::thread writing_thread([&]() {
    std::pair<int, cv::Mat> processedFrame;
    while (processedQueue.dequeue(processedFrame)) {
      cv::imshow("Frame", processedFrame.second);
      cv::waitKey(1);
    }
  });

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"CONE_DETECTED"});

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

  // state machine thread
  std::thread sm_thread([&]() {
    try {
      std::string outcome = (*sm.get())();
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR(e.what());
    }
  });

  // execute threads
  capture_thread.join();
  processing_thread.join();
  writing_thread.join();
  sm_thread.join();

  // release capture cam
  capture.release();

  return 0;
}
