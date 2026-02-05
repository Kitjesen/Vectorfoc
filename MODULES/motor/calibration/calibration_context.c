#include "calibration_context.h"
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
