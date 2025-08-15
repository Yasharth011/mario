# ifndef MARIO_HPP
# define MARIO_HPP

#include <mutex>
#include <queue> 
#include <condition_variable>

#include <yasmin/state.hpp>

struct position {
	float x_coord; 
	float y_coord; 
	float yaw; 
};

template <typename T> 
class SafeQueue { 
	public: 
		SafeQueue() : q(), m(), c() {}

		void enqueue(T t) { 
			std::lock_guard<std::mutex> lock(m); 
			q.push(t); 
			c.notify_one();
		}

		bool dequeue(T &t){ 
			std::unique_lock<std::mutex> lock(m); 
			while(q.empty()) { 
				if (finished) return false; 
				c.wait(lock); 
			}
			t = q.front(); 
			q.pop(); 
			return true; 
		}

		void setFinished(){ 
			std::lock_guard<std::mutex> lock(m); 
			finished = true; 
			c.notify_all(); 
		}

	private: 
		std::queue<T> q; 
		mutable std::mutex m; 
		std::condition_variable c; 
		bool finished = false; 
};

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

# endif
