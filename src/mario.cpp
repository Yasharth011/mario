#include <iostream>

#include <yasmin/state.hpp>
#include <yasmin/state_machine.hpp>

#include <mario.hpp>

class Navigate : public yasmin::State {

public:
  position init_pos;

  Navigate(float init_x, float init_y, float init_yaw)
      : yasmin::State({"Idle", "Arrow_Detected", "Cone_Detected"}), init_pos{init_x, init_y, init_yaw} {}

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    /* send message to nucleo  */
    /* fetch position form slam */
    return "";
  }
};

int main() { std::cout << "Hello World"; }
