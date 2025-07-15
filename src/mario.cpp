#include <iostream>

#include <librealsense2/rs.hpp>

#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin/logs.hpp> 

#include <mario.hpp>

#include <serialib.h>

class Navigate : public yasmin::State {

public:
  position init_pos;

  Navigate() : yasmin::State({"IDLE", "ARROW_DETECTED", "CONE_DETECTED"}) {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "IDLE";
  }
};

class Idle : public yasmin::State {

public:
  position init_pos;

  Idle(float init_x, float init_y, float init_yaw)
      : yasmin::State(
            {"NAVIGATE", "ARROW_DETECTED", "CONE_DETECTED", "ROTATE"}),
        init_pos{init_x, init_y, init_yaw} {};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    return "NAVIGATE";
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
  serialib serial; 

  // open serial device 
  char errorOpening = serial.openDevice(SERIAL_PORT, 9600); 

  // check for errors 
  if (errorOpening!=1) YASMIN_LOG_WARN("Error opening device");

  YASMIN_LOG_INFO("Succesful connection to %s\n",SERIAL_PORT);

  // Create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{});

  // Add states to the state machine
  sm->add_state("Navigate", std::make_shared<Navigate>(), {{"IDLE", "Idle"}});

  sm->add_state("Idle", std::make_shared<Idle>(0.0, 0.0, 0.0),
                {{"NAVIGATE", "Navigate"}});

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
  } catch (const std::exception &e) {
    std::cout << e.what();
  }

  return 0;
}
