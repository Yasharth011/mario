#include <cmath>
#include "pid.hpp"

namespace control {
struct Pid {
  struct Gains {
    double p, i, d, i_max, i_min;
  } *gains;
  struct Errors {
    double p_error_last, p_error, i_error, d_error, cmd, error_dot;
  } *errors;
};
struct Pid *initPid(double p, double i, double d, double i_max, double i_min) {
  auto pid = new struct Pid();

  pid->gains->p = p;
  pid->gains->d = d;
  pid->gains->i = i;
  pid->gains->i_max = i_max;
  pid->gains->i_min = i_min;

  pid->errors->p_error_last = 0;
  pid->errors->p_error = 0;
  pid->errors->i_error = 0;
  pid->errors->d_error = 0;
  pid->errors->cmd = 0;
  pid->errors->error_dot = 0;

  return pid;
}

void resetPid(struct Pid *pid) {
  pid->errors->p_error_last = 0.0;
  pid->errors->p_error = 0.0;
  pid->errors->i_error = 0.0;
  pid->errors->d_error = 0.0;
  pid->errors->cmd = 0.0;
}

double computeCommandErrorDot(struct Pid *pid, double error, double error_dot,
                              uint64_t dt) {

  double p_term, d_term, i_term;
  pid->errors->p_error = error; // this is error = target - state
  pid->errors->d_error = error_dot;

  if (dt == 0 || std::isnan(error) || std::isinf(error) ||
      std::isnan(error_dot) || std::isinf(error_dot)) {
    return 0.0;
  }

  // Calculate proportional contribution to command
  p_term = pid->gains->p * pid->errors->p_error;

  // Calculate the integral of the position error
  pid->errors->i_error +=
      (static_cast<double>(dt) / 1e9) * pid->errors->p_error;

  // Calculate integral contribution to command
  i_term = pid->gains->i * pid->errors->i_error;

  // Calculate derivative contribution to command
  d_term = pid->gains->d * pid->errors->d_error;

  // Compute the command
  pid->errors->cmd = p_term + i_term + d_term;

  return pid->errors->cmd;
}

double computeCommand(struct Pid *pid, double error, uint64_t dt) {
  if (dt == 0 || std::isnan(error) || std::isinf(error)) {
    return 0.0;
  }

  pid->errors->error_dot = pid->errors->d_error;

  // Calculate the derivative error
  pid->errors->error_dot =
      (error - pid->errors->p_error_last) / (static_cast<double>(dt) / 1e9);
  pid->errors->p_error_last = error;

  return computeCommandErrorDot(pid, error, pid->errors->error_dot, dt);
}
} // namespace control

