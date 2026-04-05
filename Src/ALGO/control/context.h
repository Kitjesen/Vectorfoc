// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
