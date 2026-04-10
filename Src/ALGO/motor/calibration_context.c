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
#include "config.h"
#include "rsls_calib.h"
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

  case 2: { // RSLS_CALIBRATING — reuse RSLSCalib_GetProgress() (0-100)
    // Map RSLS internal 0-100% to overall 10-70% range
    uint8_t rsls_pct = RSLSCalib_GetProgress((CalibrationContext *)ctx);
    uint32_t mapped = 10u + ((uint32_t)rsls_pct * 60u) / 100u;
    if (mapped > 70u) mapped = 70u;
    return (uint8_t)mapped;
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
