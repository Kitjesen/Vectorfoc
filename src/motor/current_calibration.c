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

#include "current_calibration.h"
#include "motor_configuration.h"       // 间接包含 board_configuration.h，提供 HW_ADC_Ix_HANDLE/JDR 宏
#include "motor_sampling.h"
#include "pwm_interface.h"
#include "error_manager.h"
#include "error_types.h"
/**
 * @file current_calibration.c
 * @brief Current offset calibration implementation
 */
/**
 * @brief Start current offset calibration
 */
CalibResult CurrentCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    ERROR_REPORT(ERROR_CALIB_INVALID_PARAMS, "Current calib: invalid params");
    return CALIB_FAILED_INVALID_PARAMS;
  }
  /* Explicit calibration runs only after the state machine accepts it. */
  /* Active braking must use the same interrupt-safe power transition as
   * voltage calibration. Startup sampling never calls this function. */
  if (MHAL_PWM_SetDuty(0.0f, 0.0f, 0.0f) != 0 ||
      !StateMachine_SetCalibrationPower(&g_ds402_state_machine, true))
    return CALIB_FAILED_INVALID_PARAMS;
  // Reset calibration context
  ctx->current.loop_count = 0;
  ctx->current.offset_sum_a = 0.0f;
  ctx->current.offset_sum_b = 0.0f;
  ctx->current.offset_sum_c = 0.0f;
  ctx->current.is_initialized = true;
  return CALIB_IN_PROGRESS;
}
/**
 * @brief Update current offset calibration
 */
CalibResult CurrentCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }
  if (!ctx->current.is_initialized) {
    return CurrentCalib_Start(motor, ctx);
  }
  CurrentCalibContext *curr = &ctx->current;
  // 统一 ADC 采样：HW_ADC_Ix_HANDLE / HW_ADC_Ix_JDR 由 board_configuration.h 定义。
  uint32_t a = HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR;
  uint32_t b = HW_ADC_IB_HANDLE.Instance->HW_ADC_IB_JDR;
  uint32_t c = HW_ADC_IC_HANDLE.Instance->HW_ADC_IC_JDR;
  /* Reject empty/unconverted and saturated 12-bit samples. */
  if (a == 0 || a >= 4095U || b == 0 || b >= 4095U || c == 0 || c >= 4095U)
    return CALIB_FAILED_INVALID_PARAMS;
  curr->offset_sum_a += (float)a;
  curr->offset_sum_b += (float)b;
  curr->offset_sum_c += (float)c;
  curr->loop_count++;
  // Check if complete
  if (curr->loop_count >= CURRENT_CALIB_CYCLES) {
    return CurrentCalib_Finish(motor, ctx);
  }
  return CALIB_IN_PROGRESS;
}
/**
 * @brief Finish current offset calibration
 */
CalibResult CurrentCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }
  CurrentCalibContext *curr = &ctx->current;
  //  ADC  (LSB)， HAL sample
  float ia = curr->offset_sum_a / (float)CURRENT_CALIB_CYCLES;
  float ib = curr->offset_sum_b / (float)CURRENT_CALIB_CYCLES;
  float ic = curr->offset_sum_c / (float)CURRENT_CALIB_CYCLES;
  ADC_SetCurrentOffsets(ia, ib, ic);
  // Clear accumulators (prepare for next calibration)
  curr->offset_sum_a = 0.0f;
  curr->offset_sum_b = 0.0f;
  curr->offset_sum_c = 0.0f;
  curr->loop_count = 0;
  curr->is_initialized = false;
  return CALIB_SUCCESS;
}

/* Startup owns this passive calibration until it completes once. */
static volatile bool s_startup_armed;
static volatile bool s_startup_ready;
static void BeginPassiveSamples(CalibrationContext *ctx) {
  ctx->current = (CurrentCalibContext){.is_initialized = true};
}
void CurrentCalib_BeginStartup(MOTOR_DATA *motor, bool hardware_ready) {
  s_startup_ready = false;
  s_startup_armed = hardware_ready && motor != NULL;
  if (motor) BeginPassiveSamples(&motor->calib_ctx);
}
void CurrentCalib_UpdateStartup(MOTOR_DATA *motor) {
  if (!s_startup_armed || s_startup_ready || motor == NULL) return;
  /* Called once per completed injected sequence, with every power output off. */
  CalibResult result = CurrentCalib_Update(motor, &motor->calib_ctx);
  if (result == CALIB_SUCCESS) {
    s_startup_ready = true;
    s_startup_armed = false; /* Finish clears is_initialized: never call it again. */
  } else if (result != CALIB_IN_PROGRESS) {
    BeginPassiveSamples(&motor->calib_ctx); /* Require consecutive valid samples. */
  }
}
bool CurrentCalib_IsReady(void) { return s_startup_ready; }
