#pragma once
#include <cinttypes>

namespace control {
struct Pid;

struct Pid *initPid(double p, double i, double d, double i_max, double i_min);

void resetPid(struct Pid *pid);

double computeCommandErrorDot(struct Pid *pid, double error, double error_dot,
                              uint64_t dt);
double computeCommand(struct Pid *pid, double error, uint64_t dt);
} // namespace control
