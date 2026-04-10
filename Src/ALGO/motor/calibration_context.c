#include "calibration_context.h"
#include "config.h"
#include <string.h>

/**
 * @file calibration_context.c
 * @brief Calibration context management implementation
 */

/**
 * @brief Initialize calibration context
 */
void CalibContext_Init(CalibrationContext *ctx) {
  if (ctx == NULL)
    return;

  // Zero out structure
  memset(ctx, 0, sizeof(CalibrationContext));

  // Initialize constants
  ctx->resistance.kI = 2.0f;

  ctx->encoder.error_array = ctx->encoder.error_array_storage;
  ctx->encoder.error_array_size = SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS;

  // Set initialization flag
  ctx->current.is_initialized = true;
}

/**
 * @brief Release calibration context resources
 */
void CalibContext_Release(CalibrationContext *ctx) {
  if (ctx == NULL)
    return;

  ctx->encoder.error_array = ctx->encoder.error_array_storage;
  ctx->encoder.error_array_size = SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS;
}

/**
 * @brief Compute calibration progress percentage (0-100)
 *
 * Stage weight allocation:
 *   CURRENT_CALIBRATING:     0-10%
 *   RSLS_CALIBRATING (R):   10-20%
 *   RSLS_CALIBRATING (L):   20-30%
 *   RSLS_CALIBRATING (DIR): 30-50%
 *   RSLS_CALIBRATING (ENC): 50-70%
 *   FLUX_CALIBRATING:       70-100%
 */
uint8_t CalibContext_GetProgress(uint8_t sub_state, uint8_t cs_state,
                                 const CalibrationContext *ctx) {
  if (ctx == NULL)
    return 0;

  // sub_state: 0=IDLE, 1=CURRENT_CALIBRATING, 2=RSLS_CALIBRATING, 3=FLUX_CALIBRATING
  // cs_state values match CS_STATE enum
  switch (sub_state) {
  case 0: // SUB_STATE_IDLE
    return 0;

  case 1: { // CURRENT_CALIBRATING
    uint32_t cycles = CURRENT_CALIB_CYCLES;
    uint32_t done = ctx->current.loop_count;
    if (done >= cycles)
      return 10;
    return (uint8_t)((done * 10u) / cycles);
  }

  case 2: { // RSLS_CALIBRATING — subdivide by CS_STATE
    // CS_MOTOR_R_*: 1,2,3  → 10-20%
    // CS_MOTOR_L_*: 4,5,6  → 20-30%
    // CS_DIR_PP_*:  7,8,9  → 30-50%
    // CS_ENCODER_*: 10,11,12,13 → 50-70%
    if (cs_state >= 1 && cs_state <= 3) { // R stage
      uint32_t cycles = RS_CALIB_CYCLES;
      uint32_t done = ctx->resistance.loop_count;
      if (done >= cycles)
        return 20;
      return (uint8_t)(10u + (done * 10u) / cycles);
    }
    if (cs_state >= 4 && cs_state <= 6) { // L stage
      uint32_t cycles = LS_CALIB_CYCLES;
      uint32_t done = ctx->inductance.loop_count;
      if (done >= cycles)
        return 30;
      return (uint8_t)(20u + (done * 10u) / cycles);
    }
    if (cs_state >= 7 && cs_state <= 9) { // DIR/PP stage
      // No fixed cycle count; use loop_count with a representative estimate
      uint32_t estimate = 10000u; // ~0.5s @ 20kHz
      uint32_t done = ctx->dir_pole.loop_count;
      if (done >= estimate)
        return 50;
      return (uint8_t)(30u + (done * 20u) / estimate);
    }
    if (cs_state >= 10 && cs_state <= 13) { // ENC stage
      uint32_t total = (uint32_t)(SAMPLES_PER_POLE_PAIR * MAX_POLE_PAIRS);
      int16_t done = ctx->encoder.sample_count;
      if (done < 0)
        done = 0;
      if ((uint32_t)done >= total)
        return 70;
      return (uint8_t)(50u + ((uint32_t)done * 20u) / total);
    }
    return 50; // fallback mid-point
  }

  case 3: { // FLUX_CALIBRATING
    uint32_t cycles = FLUX_CALIB_CYCLES;
    uint32_t done = ctx->flux.loop_count;
    if (done >= cycles)
      return 100;
    return (uint8_t)(70u + (done * 30u) / cycles);
  }

  default:
    return 0;
  }
}

/**
 * @brief Reset calibration context
 */
void CalibContext_Reset(CalibrationContext *ctx) {
  if (ctx == NULL)
    return;

  // Release resources first
  CalibContext_Release(ctx);

  // Re-initialize
  CalibContext_Init(ctx);
}
