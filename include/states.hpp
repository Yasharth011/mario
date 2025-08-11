# ifndef STATES_HPP 
# define STATES_HPP

#include <yasmin/state.hpp>

class Navigate : public yasmin::State {

public:
  serialib serial;

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override;
};

class Idle : public yasmin::State {

public:
  serialib serial;


  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override;
};

class Cone_Detected : public yasmin::State {

public:
  serialib serial;

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override;
};

class Arrow_Detected : public yasmin::State {

public:
  serialib serial;

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override; 
};
# endif
