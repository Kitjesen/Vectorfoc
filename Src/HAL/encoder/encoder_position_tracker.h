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

#ifndef ENCODER_POSITION_TRACKER_H
#define ENCODER_POSITION_TRACKER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Update a multi-turn count from a wrapped single-turn sample.
 *
 * The first valid sample establishes the boot-relative origin and therefore
 * contributes zero delta. Subsequent samples use shortest-path wrap handling.
 */
static inline int32_t EncoderPosition_Update(int32_t count, int32_t cpr,
                                             bool *initialized,
                                             int32_t *last_count,
                                             int64_t *shadow_count) {
  if (initialized == NULL || last_count == NULL || shadow_count == NULL ||
      cpr <= 1) {
    return 0;
  }

  if (!*initialized) {
    *last_count = count;
    *shadow_count = 0;
    *initialized = true;
    return 0;
  }

  int32_t delta = count - *last_count;
  int32_t half_cpr = cpr / 2;
  if (delta > half_cpr) {
    delta -= cpr;
  } else if (delta < -half_cpr) {
    delta += cpr;
  }

  *last_count = count;
  *shadow_count += (int64_t)delta;
  return delta;
}

#endif /* ENCODER_POSITION_TRACKER_H */
