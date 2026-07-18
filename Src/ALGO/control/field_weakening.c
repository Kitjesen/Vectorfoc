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

#include "field_weakening.h"
#include "common.h"
#include "config.h"
#include <float.h>
#include <stdbool.h>
#include <stdint.h>

static float s_id_fw_integral = 0.0f;

static bool FieldWeakening_IsFinite(float value) {
  union {
    float f;
    uint32_t u;
  } bits;
  bits.f = value;
  return (bits.u & 0x7F800000u) != 0x7F800000u;
}

static float FieldWeakening_CalcLinear(const FieldWeakening_Config_t *config,
                                       float velocity) {
  if (!FieldWeakening_IsFinite(config->start_velocity) ||
      !FieldWeakening_IsFinite(velocity) || config->start_velocity <= 0.0f)
    return 0.0f;

  float abs_velocity = fabsf(velocity);
  if (abs_velocity <= config->start_velocity)
    return 0.0f;

  float ratio =
      (abs_velocity - config->start_velocity) / config->start_velocity;
  ratio = CLAMP(ratio, 0.0f, 1.0f);
  return -config->max_weakening_current * ratio;
}

static float FieldWeakening_CalcVoltageSaturation(
    const MOTOR_DATA *motor, const FieldWeakening_Config_t *config, float dt) {
  if (!FieldWeakening_IsFinite(dt) || dt <= 0.0f || dt > 0.1f)
    return 0.0f;

  const float weakening_ramp_rate = 100.0f; /* A/s */
  if (motor->algo_output.voltage_saturated) {
    s_id_fw_integral -= weakening_ramp_rate * dt;
  } else if (s_id_fw_integral < 0.0f) {
    s_id_fw_integral += weakening_ramp_rate * 0.1f * dt;
  }

  s_id_fw_integral =
      CLAMP(s_id_fw_integral, -config->max_weakening_current, 0.0f);
  return s_id_fw_integral;
}

float FieldWeakening_Calculate(const MOTOR_DATA *motor,
                               const FieldWeakening_Config_t *config,
                               float dt) {
  if (motor == NULL || config == NULL ||
      !FieldWeakening_IsFinite(config->max_weakening_current) ||
      !FieldWeakening_IsFinite(config->start_velocity) ||
      !FieldWeakening_IsFinite(motor->feedback.velocity) ||
      config->max_weakening_current <= 0.0f) {
    s_id_fw_integral = 0.0f;
    return 0.0f;
  }
  if (!FieldWeakening_IsFinite(dt) || dt <= 0.0f || dt > 0.1f) {
    return 0.0f;
  }

  float linear = FieldWeakening_CalcLinear(config, motor->feedback.velocity);
  float voltage_saturation =
      FieldWeakening_CalcVoltageSaturation(motor, config, dt);
  return linear < voltage_saturation ? linear : voltage_saturation;
}

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *config) {
  (void)FieldWeakening_Calculate(motor, config, CURRENT_MEASURE_PERIOD);
}

void FieldWeakening_Reset(void) { s_id_fw_integral = 0.0f; }
