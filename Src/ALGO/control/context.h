#ifndef CONTROL_PRIVATE_H
#define CONTROL_PRIVATE_H

#include "motor.h"
#include "foc/foc_algorithm.h"
#include "trajectory/trap_traj.h"


/**
 * @brief Control module internal context
 */
typedef struct {
  // Outer loop decimation counter (20kHz -> 5kHz)
  uint8_t loop_count;

  // Mode switch detection
  CONTROL_MODE last_mode;

  // Setpoint caches
  float vel_set; // Internal velocity setpoint cache
  float iq_setpoint_cache;
  float id_setpoint_cache;

  // Trajectory planning state
  bool trajectory_active;
  float trajectory_time;
  TrajTypeDef traj;

  // FOC Algorithm State & Config have been moved to MOTOR_DATA
  // to allow global access for debugging/logging (VOFA).
} MotorControlCtx;

#endif // CONTROL_PRIVATE_H
