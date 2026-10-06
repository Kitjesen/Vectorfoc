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

void Feedforward_Update(MOTOR_DATA *motor) {
  if (motor == NULL) {
    return;
  }
  Feedforward_Config_t config = {
      .inertia = motor->Controller.inertia,
      .friction = motor->advanced.ff_friction,
  };
  float torque = Feedforward_AlgorithmUpdate(
      &s_feedforward_state, &config, motor->Controller.vel_setpoint,
      TORQUE_COMPENSATION_UPDATE_PERIOD_S);
  motor->Controller.input_torque += torque;
}

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *config) {
  if (motor == NULL || config == NULL) {
    return;
  }
  /* Called from every current-control cycle; use the 20 kHz FOC step. */
  float id_fw = FieldWeakening_AlgorithmUpdate(
      &s_field_weakening_state, config, motor->feedback.velocity,
      motor->algo_output.voltage_saturated, FOC_LOOP_PERIOD_S);
  if (id_fw == 0.0f) {
    return;
  }
  float id_ref = motor->algo_input.Id_ref + id_fw;
  if (id_ref < -config->max_weakening_current) {
    id_ref = -config->max_weakening_current;
  }
  if (id_ref > config->max_weakening_current) {
    id_ref = config->max_weakening_current;
  }
  motor->algo_input.Id_ref = id_ref;
}

void FieldWeakening_Reset(void) {
  FieldWeakening_AlgorithmReset(&s_field_weakening_state);
}
