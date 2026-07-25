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

#include "feedforward.h"
#include "config.h"
#include "torque_utils.h"

static float s_last_velocity_ref = 0.0f;
static float s_torque_ff = 0.0f;
static bool s_has_last = false;

void Feedforward_Init(Feedforward_Params_t *params) {
  if (params == NULL)
    return;
  params->inertia = 0.0f;
  params->friction_coeff = 0.0f;
}

void Feedforward_Reset(void) {
  s_last_velocity_ref = 0.0f;
  s_torque_ff = 0.0f;
  s_has_last = false;
}

void Feedforward_Update(MOTOR_DATA *motor, const Feedforward_Params_t *params) {
  if (motor == NULL || params == NULL)
    return;

  CONTROL_MODE mode = motor->state.Control_Mode;
  bool closed_loop_motion =
      mode == CONTROL_MODE_VELOCITY || mode == CONTROL_MODE_POSITION ||
      mode == CONTROL_MODE_VELOCITY_RAMP ||
      mode == CONTROL_MODE_POSITION_RAMP;
  if (!closed_loop_motion) {
    s_torque_ff = 0.0f;
    s_has_last = false;
    return;
  }

  float velocity_ref = motor->Controller.vel_setpoint;
  float dt = 1.0f / (float)ADV_CONTROL_HZ;
  float acceleration_ref = 0.0f;
  if (s_has_last && dt > 0.0f) {
    acceleration_ref = (velocity_ref - s_last_velocity_ref) / dt;
  }
  s_last_velocity_ref = velocity_ref;
  s_has_last = true;

  /* Ramp modes already provide trajectory inertia torque_setpoint. */
  bool trajectory_supplies_inertia =
      mode == CONTROL_MODE_VELOCITY_RAMP ||
      mode == CONTROL_MODE_POSITION_RAMP;
  float inertia_torque =
      trajectory_supplies_inertia
          ? 0.0f
          : Control_InertiaTorque(params->inertia, acceleration_ref);
  float viscous_torque = params->friction_coeff * velocity_ref;

  /* Store an independent compensation. Never rewrite the user command. */
  s_torque_ff = inertia_torque + viscous_torque;
}

float Feedforward_GetTorque(void) { return s_torque_ff; }

float Feedforward_GetCurrent(const MOTOR_DATA *motor) {
  if (motor == NULL) {
    return 0.0f;
  }
  float current = 0.0f;
  (void)Control_TorqueToCurrent(s_torque_ff, motor->Controller.torque_const,
                                &current);
  return current;
}
