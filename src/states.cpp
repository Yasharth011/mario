#include <states.hpp> 

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
