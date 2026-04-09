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

#include "rate_limiter.h"
#include "common.h"
#include <math.h>
// init
void RateLimiter_Init(RateLimiterTypeDef *limiter, float max_rate) {
  if (limiter == NULL)
    return;
  limiter->last_value = 0.0f;
  limiter->max_rate = fabsf(max_rate);
  limiter->initialized = false;
}
//
void RateLimiter_Reset(RateLimiterTypeDef *limiter, float initial_value) {
  if (limiter == NULL)
    return;
  limiter->last_value = initial_value;
  limiter->initialized = true;
}
//
float RateLimiter_Apply(RateLimiterTypeDef *limiter, float input, float dt) {
  if (limiter == NULL)
    return input;
  if (dt < 0.000001f || dt > 1.0f)
    return limiter->last_value;
  // First call: pass through
  if (!limiter->initialized) {
    limiter->last_value = input;
    limiter->initialized = true;
    return input;
  }
  // calc
  float max_delta = limiter->max_rate * dt;
  // calcactual
  float delta = input - limiter->last_value;
  // limit
  float limited_delta = CLAMP(delta, -max_delta, max_delta);
  // update last_value
  limiter->last_value += limited_delta;
  return limiter->last_value;
}
// set
void RateLimiter_SetMaxRate(RateLimiterTypeDef *limiter, float max_rate) {
  if (limiter == NULL)
    return;
  limiter->max_rate = fabsf(max_rate);
}
// get
float RateLimiter_GetValue(const RateLimiterTypeDef *limiter) {
  if (limiter == NULL)
    return 0.0f;
  return limiter->last_value;
}
