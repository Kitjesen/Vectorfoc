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
 * @file    trigonometry.h
 * @brief   Fast trigonometric functions for real-time control.
 * @details Taylor series approximation (~5x faster than math.h, ~0.1% error).
 */

#ifndef FOC_TRIGONOMETRY_H
#define FOC_TRIGONOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Fast sin/cos calculation (Taylor series).
 * @param  angle    [rad] Angle.
 * @param  sin_val  [out] sin(angle).
 * @param  cos_val  [out] cos(angle).
 * @note   Accuracy: ~0.1% error, Speed: ~5x faster than math.h.
 */
void Trig_FastSinCos(float angle, float *sin_val, float *cos_val);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRIGONOMETRY_H */
