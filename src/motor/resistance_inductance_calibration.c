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

#include "resistance_inductance_calibration.h"
#include "encoder_calibration.h"
#include "motor_configuration.h"   // 间接包含 board_configuration.h → HW_POSITION_SENSOR_MODE
#include "position_sensor.h"
#include "foc/control_dispatcher.h"
#include "pwm_interface.h"
#include <math.h>
#include <string.h>

static CalibResult ResistanceCalib_Update(MOTOR_DATA *motor,
                                   ResistanceCalibContext *ctx, float dt);
static CalibResult InductanceCalib_Update(MOTOR_DATA *motor,
                                   InductanceCalibContext *ctx, float dt);

/**
 * @file resistance_inductance_calibration.c
 * @brief Rs/Ls/Enc Calibration Coordinator (Refactored)
 *
 * Coordinates sub-modules for complete motor parameter calibration
 */

/**
 * @brief Initialize Rs/Ls calibration
 */
CalibResult RSLSCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  // Reset all sub-contexts
  memset(&ctx->resistance, 0, sizeof(ResistanceCalibContext));
  memset(&ctx->inductance, 0, sizeof(InductanceCalibContext));
  memset(&ctx->dir_pole, 0, sizeof(DirectionPoleCalibContext));
  /* Calibration transient reset must retain the fixed CCM workspaces. */
  int *error_array = ctx->encoder.error_array;
  size_t error_array_size = ctx->encoder.error_array_size;
  int16_t *offset_lut = ctx->encoder.offset_lut;
  size_t offset_lut_size = ctx->encoder.offset_lut_size;
  memset(&ctx->encoder, 0, sizeof(EncoderCalibContext));
  ctx->encoder.error_array = error_array;
  ctx->encoder.error_array_size = error_array_size;
  ctx->encoder.offset_lut = offset_lut;
  ctx->encoder.offset_lut_size = offset_lut_size;

  // Initialize constants (Resistance)
  ctx->resistance.kI = 2.0f;

  // Initialize constants (Inductance)
  ctx->inductance.voltages[0] = -VOLTAGE_MAX_CALIB;
  ctx->inductance.voltages[1] = +VOLTAGE_MAX_CALIB;

  const PositionSensorDescriptor_t *sensor = PositionSensor_GetDescriptor();
  if (sensor == NULL ||
      ((sensor->capabilities & POSITION_SENSOR_CAP_RAW_DIRECTION_POLE) != 0u &&
       PositionSensor_RawCalibrationPrepareClockwise() != POSITION_SENSOR_STATUS_OK))
    return CALIB_FAILED_INVALID_PARAMS;

  // State machine starting point
  motor->state.Cs_State = CS_MOTOR_R_START;

  return CALIB_IN_PROGRESS;
}

/**
 * @brief Update Rs/Ls calibration state machine
 */
CalibResult RSLSCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx,
                             float dt) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  CalibResult result = CALIB_IN_PROGRESS;

  // Execute sub-state machine based on current calibration state
  switch (motor->state.Cs_State) {
  // --- Resistance Calibration ---
  case CS_MOTOR_R_START:
  case CS_MOTOR_R_LOOP:
  case CS_MOTOR_R_END:
    result = ResistanceCalib_Update(motor, &ctx->resistance, dt);
    // If sub-module returns SUCCESS, it means that node is done, but overall
    // flow might continue Our sub-modules switch Cs_State to next stage in
    // their END state and return SUCCESS So if SUCCESS, we verify and return
    // IN_PROGRESS to main loop (to call next state next frame)
    if (result == CALIB_SUCCESS) {
      // Logic has switched state internally, return IN_PROGRESS so loop
      // continues into next case next time
      result = CALIB_IN_PROGRESS;
    }
    break;

  // --- Inductance Calibration ---
  case CS_MOTOR_L_START:
  case CS_MOTOR_L_LOOP:
  case CS_MOTOR_L_END:
    result = InductanceCalib_Update(motor, &ctx->inductance, dt);
    if (result == CALIB_SUCCESS)
      result = CALIB_IN_PROGRESS;
    break;

  // --- Direction and Pole Pair ---
  case CS_DIR_PP_START:
  case CS_DIR_PP_LOOP:
  case CS_DIR_PP_END:
    result = DirectionPoleCalib_Update(motor, &ctx->dir_pole);
    if (result == CALIB_SUCCESS)
      result = CALIB_IN_PROGRESS;
    break;

  // --- Encoder Calibration ---
  case CS_ENCODER_START:
  case CS_ENCODER_CW_LOOP:
  case CS_ENCODER_CCW_LOOP:
  case CS_ENCODER_END:
  case CS_REPORT_OFFSET_LUT:
    result = EncoderCalib_Update(motor, &ctx->encoder);
    // If SUCCESS here, the entire RSLS calibration is truly done
    break;

  default:
    result = CALIB_FAILED_INVALID_PARAMS;
    break;
  }

  return result;
}

/**
 * @brief Get calibration progress
 */
uint8_t RSLSCalib_GetProgress(CalibrationContext *ctx) {
  if (ctx == NULL)
    return 0;

  if (ctx->resistance.loop_count > 0 &&
      ctx->resistance.loop_count < RS_CALIB_CYCLES) {
    return (uint8_t)(ctx->resistance.loop_count * 20 / RS_CALIB_CYCLES);
  }

  if (ctx->inductance.loop_count > 0 &&
      ctx->inductance.loop_count < (LS_CALIB_CYCLES * 2)) {
    return 20 +
           (uint8_t)(ctx->inductance.loop_count * 20 / (LS_CALIB_CYCLES * 2));
  }

  if (ctx->dir_pole.loop_count > 0) {
    // Hard to estimate precisely, giving rough range
    return 40 + (uint8_t)(CLAMP(ctx->dir_pole.loop_count, 0, 100) * 20 / 100);
  }

  if (ctx->encoder.sample_count > 0) {
    // Rough estimate
    return 60 +
           (uint8_t)(CLAMP(ctx->encoder.sample_count, 0, 1000) * 40 / 1000);
  }

  return 0;
}

/* Electrical identification steps are private to this state machine. */
static CalibResult ResistanceCalib_Update(MOTOR_DATA *motor,
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

static CalibResult InductanceCalib_Update(MOTOR_DATA *motor,
                                   InductanceCalibContext *ctx, float dt) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }
  switch (motor->state.Cs_State) {
  case CS_MOTOR_L_START:
    ctx->loop_count = 0;
    ctx->Ialphas[0] = 0.0f;
    ctx->Ialphas[1] = 0.0f;
    // Initialize voltages (if not initialized)
    if (ctx->voltages[0] == 0.0f && ctx->voltages[1] == 0.0f) {
      ctx->voltages[0] = -VOLTAGE_MAX_CALIB;
      ctx->voltages[1] = +VOLTAGE_MAX_CALIB;
    }
    Control_InjectVoltage(motor, ctx->voltages[0], 0.0f, 0.0f);
    motor->state.Cs_State = CS_MOTOR_L_LOOP;
    return CALIB_IN_PROGRESS;
  case CS_MOTOR_L_LOOP: {
    int i = ctx->loop_count & 1; // Alternate between 0 and 1
    // Apply test voltage
    Control_InjectVoltage(motor, ctx->voltages[i], 0.0f, 0.0f);
    // Record only final steady state current
    if (ctx->loop_count >= LS_CALIB_CYCLES - 5) {
      ctx->Ialphas[i] += motor->algo_input.Ia / 5.0f;
    }
    ctx->loop_count++;
    if (ctx->loop_count >= (LS_CALIB_CYCLES << 1)) {
      MHAL_PWM_Brake();
      motor->state.Cs_State = CS_MOTOR_L_END;
    }
    return CALIB_IN_PROGRESS;
  }
  case CS_MOTOR_L_END: {
    // [FIX] 电感计算公式修正：L = V * dt / dI
    // 原公式 dI_by_dt = (I1 - I0) / dt 是错误的，因为 dt 是单个周期时间
    // 正确做法：dI = I1 - I0（电流变化量），dt = 单周期时间
    // L = V / (dI/dt) = V * dt / dI
    float dI = ctx->Ialphas[1] - ctx->Ialphas[0];

    // Division by zero protection
    if (fabsf(dI) < 1e-3f) {
      motor->parameters.Ls = 0.00003f; // Use default value
    } else {
      // L = V * dt / dI，其中 dt 是单个测量周期
      // 由于是交替测量，实际 dt = 2 * CURRENT_SAMPLE_PERIOD_S * LS_CALIB_CYCLES
      // 但这里 dI 是稳态电流差，不是瞬态变化
      // 正确公式：L = (2 * V_test) / (dI / dt_settle)
      // 简化：使用经验公式 L = V / (dI/dt) * 2/3（相电感到线电感转换）
      float dI_by_dt = dI / dt;
      float L = (2.0f * VOLTAGE_MAX_CALIB) / fabsf(dI_by_dt);
      motor->parameters.Ls = L * 2.0f / 3.0f;
    }
    // Update current loop gains
    Control_UpdateCurrentGains(motor);
    // Next step is direction/pole pair calibration
    motor->state.Cs_State = CS_DIR_PP_START;
    return CALIB_SUCCESS;
  }
  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}
