#include "current_calib.h"
#include "config.h"
#include "motor_adc.h"
#include "hal_pwm.h" // For MHAL_PWM_Brake (统一 HAL)
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

  // Accumulate ADC samples
  curr->offset_sum_a += (float)(ADC1->JDR3);
  curr->offset_sum_b += (float)(ADC1->JDR2);
  curr->offset_sum_c += (float)(ADC1->JDR1);

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

  // 写入 ADC 层零偏 (LSB)，供 HAL 采样使用
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
