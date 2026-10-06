#ifndef CONTROL_TEST_PROBE_H
#define CONTROL_TEST_PROBE_H

#include "command_limiter.h"
#include "trapezoid_trajectory.h"

/* Read-only access to state for the original/current implementation comparison. */
typedef struct {
  unsigned velocity_loop_count;
  unsigned position_loop_count;
  float vel_set;
  TrajTypeDef traj;
  RateLimiterTypeDef velocity;
  RateLimiterTypeDef torque;
} ControlTestSnapshot;

void ControlTest_ReadSnapshot(ControlTestSnapshot *snapshot);

#endif
