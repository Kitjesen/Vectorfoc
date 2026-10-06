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

#ifndef CONTROL_CONTEXT_H
#define CONTROL_CONTEXT_H

#include "control_mode.h"
#include "algorithm/ladrc_controller.h"
#include "algorithm/pid_controller.h"
#include "algorithm/trapezoid_trajectory.h"
#include <stdbool.h>
#include <stdint.h>

/* State owned by the runtime dispatcher; trajectory planning stays here so
 * the outer loop does not need to know about MOTOR_DATA. */
typedef struct {
  uint16_t velocity_loop_tick_count;
  uint16_t position_loop_tick_count;
  float vel_set;
  TrajTypeDef traj;
} MotorControlCtx;

/* Numeric snapshot passed from the runtime object into the outer controller.
 * PID/LADRC state remains mutable through these narrow algorithm pointers. */
typedef struct {
  CONTROL_MODE mode;
  float feedback_position;
  float feedback_velocity;
  float input_position;
  float input_velocity;
  float vel_limit;
  float pos_setpoint;
  float vel_setpoint;
  float torque_setpoint;
  PidTypeDef *position_pid;
  PidTypeDef *velocity_pid;
  LADRC_Config_t *ladrc_config;
  LADRC_State_t *ladrc_state;
  float ladrc_enable;
  bool params_updated;
  float vel_feedback_filtered;
  bool vel_filter_initialized;
} OuterLoopInput;

typedef struct {
  float iq_ref;
  float id_ref;
  float vel_feedback_filtered;
  bool vel_filter_initialized;
} OuterLoopOutput;

/* Position and velocity loops have independent periods and are called by the
 * dispatcher only when their respective decimation counters expire. */
void Control_UpdatePositionLoop(const OuterLoopInput *input, MotorControlCtx *ctx);
void Control_UpdateVelocityLoop(const OuterLoopInput *input,
                                OuterLoopOutput *output, MotorControlCtx *ctx);

#endif /* CONTROL_CONTEXT_H */
