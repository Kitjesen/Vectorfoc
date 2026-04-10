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

#include "calib_resistance.h"
#include "config.h"
#include "control/impl.h" // For Control_InjectVoltage
#include "hal_pwm.h" // For MHAL_PWM_Brake ( HAL)
#include <math.h>
/**
 * @file calib_resistance.c
 * @brief Resistance calibration implementation
 */
CalibResult ResistanceCalib_Update(MOTOR_DATA *motor,
                                   ResistanceCalibContext *ctx, float dt) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }
  switch (motor->state.Cs_State) {
  case CS_MOTOR_R_START:
    ctx->loop_count = 0;
    ctx->voltage = 0.0f;
    // Initialize parameters (use default if kI is not set)
    if (ctx->kI == 0.0f)
      ctx->kI = 2.0f;
    motor->state.Cs_State = CS_MOTOR_R_LOOP;
    return CALIB_IN_PROGRESS;
  case CS_MOTOR_R_LOOP:
    // PI controller drives current to target
    // error = target - measured
    // integral += Ki * dt * error
    ctx->voltage +=
        ctx->kI * dt * (CURRENT_MAX_CALIB - motor->algo_input.Ia);
    // Apply test voltage along Phase A (SVPWM angle=0)
    Control_InjectVoltage(motor, ctx->voltage, 0, 0);
    ctx->loop_count++;
    if (ctx->loop_count >= RS_CALIB_CYCLES) {
      MHAL_PWM_Brake();
      motor->state.Cs_State = CS_MOTOR_R_END;
    }
    return CALIB_IN_PROGRESS;
  case CS_MOTOR_R_END:
    // Calculate resistance: R = V / I * (2/3)
    // Prevent division by zero
    if (CURRENT_MAX_CALIB > 1e-4f) {
      motor->parameters.Rs = (ctx->voltage / CURRENT_MAX_CALIB) * (2.0f / 3.0f);
    } else {
      motor->parameters.Rs = 0.0f; // Alternatively set to default
    }
    // Next step is usually inductance calibration
    motor->state.Cs_State = CS_MOTOR_L_START;
    return CALIB_SUCCESS;
  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}
