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

/**
 * @file    rate_limiter.h
 * @brief   Rate limiter to prevent command spikes.
 * @details
 * - Context: Limits the rate of change of control commands.
 * - Units:   Command rate [units/s].
 */

#ifndef ALGORITHM_RATE_LIMITER_H
#define ALGORITHM_RATE_LIMITER_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Rate limiter state.
 */
typedef struct {
  float last_value; /**< Last output value */
  float max_rate;   /**< [units/s] Max rate of change */
  bool initialized; /**< Flag: true if initialized */
} RateLimiterTypeDef;

/**
 * @brief  Initialize rate limiter.
 * @param  limiter   Limiter instance.
 * @param  max_rate  [units/s] Maximum rate of change (>0).
 */
void RateLimiter_Init(RateLimiterTypeDef *limiter, float max_rate);

/**
 * @brief  Reset rate limiter to a specific value.
 * @param  limiter        Limiter instance.
 * @param  initial_value  Starting value for next call.
 */
void RateLimiter_Reset(RateLimiterTypeDef *limiter, float initial_value);

/**
 * @brief  Apply rate limiting.
 * @param  limiter  Limiter instance.
 * @param  input    Desired input value.
 * @param  dt       [s] Time step.
 * @return Rate-limited output value.
 * @note   First call returns input directly.
 */
float RateLimiter_Apply(RateLimiterTypeDef *limiter, float input, float dt);

/**
 * @brief  Update maximum rate.
 * @param  limiter   Limiter instance.
 * @param  max_rate  [units/s] New maximum rate of change.
 */
void RateLimiter_SetMaxRate(RateLimiterTypeDef *limiter, float max_rate);

/**
 * @brief  Get current output value.
 * @param  limiter  Limiter instance.
 * @return Current output value.
 */
float RateLimiter_GetValue(const RateLimiterTypeDef *limiter);

#ifdef __cplusplus
}
#endif

#endif // ALGORITHM_RATE_LIMITER_H
