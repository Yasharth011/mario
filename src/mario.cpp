#include <iostream>

#include <librealsense2/rs.hpp>

#include <yasmin/logs.hpp>
#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>

#include <mario.hpp>

#include <serialib.h>

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

  Cone_Detected(serialib x) : yasmin::State({"NAVIGATE", "IDLE"}), serial(x) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "Idle";
  }
};

int main(int argc, cahr *argv[]) {

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

  // pipe.start(cfg);

  rs2::pointcloud pc;
  rs2::points points;

  // serial object
  serialib nucleo_com;

  // open serial device
  char errorOpening = serial.openDevice(SERIAL_PORT, 9600);

  // check for errors
  if (errorOpening != 1)
    YASMIN_LOG_ERROR("Error opening device");

  YASMIN_LOG_INFO("Succesful connection to %s\n", SERIAL_PORT);

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{});

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
                {{"IDLE", "Idle"}, {"NAVIAGE", "Navigate"}});

  sm->add_state("Cone_Detected", std::make_shared<Cone_Detected()>(nucleo_com),
                {{"IDLE", "Idle"}});

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR(e.what())
  }

  return 0;
}
