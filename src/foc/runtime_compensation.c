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

#include "runtime_compensation.h"
#include "algorithm/torque_feedforward.h"
#include "algorithm/field_weakening.h"
#include "motor_configuration.h"

static Feedforward_State_t s_feedforward_state;
static FieldWeakening_State_t s_field_weakening_state;
static float s_feedforward_torque;

void Feedforward_Update(MOTOR_DATA *motor) {
  if (motor == NULL) {
    return;
  }
  CONTROL_MODE mode = motor->state.Control_Mode;
  bool motion_mode = mode == CONTROL_MODE_VELOCITY ||
      mode == CONTROL_MODE_POSITION || mode == CONTROL_MODE_VELOCITY_RAMP ||
      mode == CONTROL_MODE_POSITION_RAMP;
  if (!motion_mode) {
    Feedforward_Reset();
    return;
  }
  bool trajectory_inertia = mode == CONTROL_MODE_VELOCITY_RAMP ||
      mode == CONTROL_MODE_POSITION_RAMP;
  Feedforward_Config_t config = {
      .inertia = trajectory_inertia ? 0.0f : motor->Controller.inertia,
      .friction = motor->advanced.ff_friction,
  };
  float torque = Feedforward_AlgorithmUpdate(
      &s_feedforward_state, &config, motor->Controller.vel_setpoint,
      TORQUE_COMPENSATION_UPDATE_PERIOD_S);
  s_feedforward_torque = torque;
}

float FieldWeakening_Calculate(const MOTOR_DATA *motor,
                               const FieldWeakening_Config_t *config, float dt_s) {
  if (motor == NULL || config == NULL) return 0.0f;
  return FieldWeakening_AlgorithmUpdate(&s_field_weakening_state, config,
      motor->feedback.velocity, motor->algo_output.voltage_saturated, dt_s);
}

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *config) {
  (void)FieldWeakening_Calculate(motor, config, FOC_LOOP_PERIOD_S);
}

void FieldWeakening_Reset(void) {
  FieldWeakening_AlgorithmReset(&s_field_weakening_state);
}

void Feedforward_Reset(void) {
  s_feedforward_torque = 0.0f;
  Feedforward_AlgorithmReset(&s_feedforward_state);
}
float Feedforward_GetCurrent(const MOTOR_DATA *motor) {
  float current = 0.0f;
  if (motor != NULL)
    (void)Control_TorqueToCurrent(s_feedforward_torque,
                                  motor->Controller.torque_const, &current);
  return current;
}
