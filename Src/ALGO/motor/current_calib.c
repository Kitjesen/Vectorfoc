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

#include "current_calib.h"
#include "config.h"       // 间接包含 board_config.h，提供 HW_ADC_Ix_HANDLE/JDR 宏
#include "motor_adc.h"
#include "hal_pwm.h"
#include "error_manager.h"
#include "error_types.h"
/**
 * @file current_calib.c
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
  // Ensure PWM is off (all low sides on)
  MHAL_PWM_Brake();
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
  // 统一 ADC 采样：HW_ADC_Ix_HANDLE / HW_ADC_Ix_JDR 由 board_config.h 定义，
  // VectorFOC 三相均在 ADC1；X-STAR IB 在 ADC2，通过宏透明处理，此处无 #ifdef。
  curr->offset_sum_a += (float)(HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR);
  curr->offset_sum_b += (float)(HW_ADC_IB_HANDLE.Instance->HW_ADC_IB_JDR);
  curr->offset_sum_c += (float)(HW_ADC_IC_HANDLE.Instance->HW_ADC_IC_JDR);
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
