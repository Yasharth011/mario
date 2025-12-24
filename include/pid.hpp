#pragma once
#include <cinttypes>
#include <chrono>
#include <cstdint>

namespace control {
struct Pid {
  struct Gains {
    double p, i, d;
  } *gains;
  struct Errors {
    double p_error_last, p_error, i_error, d_error, cmd, error_dot;
  } *errors;
};

struct Pid *initPid(double p, double i, double d);

void resetPid(struct Pid *pid);

double computeCommandErrorDot(struct Pid *pid, double error, double error_dot,
                              uint64_t dt);
double computeCommand(struct Pid *pid, double error, uint64_t dt);
} // namespace control
