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
 * @file    foc_transforms.h
 * @brief   Common math utilities and constants for FOC.
 */

#ifndef FOC_TRANSFORMS_H
#define FOC_TRANSFORMS_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pure coordinate transforms, modulation and fast trigonometry. */
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha,
                      float *Ibeta);
void Clarke_Inverse(float Valpha, float Vbeta, float *Va, float *Vb, float *Vc);
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id,
                    float *Iq);
void Park_Inverse(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);
/* Returns 0 for linear modulation, 1 when the vector was scaled to fit,
 * and -1 when Vbus is invalid. */
int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc);
void Trig_FastSinCos(float angle, float *sin_val, float *cos_val);

/* Math constants */
#define MATH_PI (3.141592653589793f)
#define MATH_2PI (6.283185307179586f)
#define MATH_SQRT3 (1.732050807568877f)
#define MATH_ONE_BY_SQRT3 (0.577350269189626f)
#define MATH_SQRT3_BY_2 (0.866025403784439f)
#define MATH_2_BY_3 (0.666666666666667f)

/* Open-loop electrical phase uses [0, 2*pi), unlike signed angle errors.
 * Keep the previous encoder helper's arithmetic and negative-zero behavior. */
static inline float Math_WrapAnglePositive(float angle) {
  float a = fmodf(angle, MATH_2PI);
  return a >= 0.0f ? a : (a + MATH_2PI);
}

/**
 * @brief  Normalize angle to [-π, π].
 * @param  angle [rad] Input angle.
 * @return [rad] Normalized angle.
 * @note   Uses fmodf for O(1) complexity instead of while loops.
 */
static inline float Math_NormalizeAngle(float angle) {
  /* Fast path for common case */
  if (angle >= -MATH_PI && angle <= MATH_PI) {
    return angle;
  }
  /* Use fmodf for O(1) normalization */
  angle = fmodf(angle + MATH_PI, MATH_2PI);
  if (angle < 0.0f) {
    angle += MATH_2PI;
  }
  return angle - MATH_PI;
}

/**
 * @brief  Clamp value to [min, max].
 * @param  value Input value.
 * @param  min   Minimum limit.
 * @param  max   Maximum limit.
 * @return Clamped value.
 */
static inline float Math_Clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

/**
 * @brief  Sign function.
 * @param  value Input value.
 * @return 1.0f for nonnegative input, or -1.0f for negative input.
 */
static inline float Math_Sign(float value) {
  return (value >= 0.0f) ? 1.0f : -1.0f;
}

/**
 * @brief  Absolute value.
 * @param  value Input value.
 * @return Absolute value.
 */
static inline float Math_Abs(float value) {
  return (value >= 0.0f) ? value : -value;
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRANSFORMS_H */
