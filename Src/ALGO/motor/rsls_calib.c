#include "rsls_calib.h"
#include "calib_encoder.h"
#include "calib_inductance.h"
#include "calib_resistance.h"
#include "mt6816_encoder.h"

#include "config.h"
#include <malloc.h>
#include <string.h>

/**
 * @file rsls_calib.c
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
  memset(&ctx->encoder, 0, sizeof(EncoderCalibContext));

  // Initialize constants (Resistance)
  ctx->resistance.kI = 2.0f;

  // Initialize constants (Inductance)
  ctx->inductance.voltages[0] = -VOLTAGE_MAX_CALIB;
  ctx->inductance.voltages[1] = +VOLTAGE_MAX_CALIB;

  // Set initial direction
  MT6816_Handle_t *enc = (MT6816_Handle_t *)motor->components.encoder;
  enc->dir = MT6816_DIR_CW;

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
