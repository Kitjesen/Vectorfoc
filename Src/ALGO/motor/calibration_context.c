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
