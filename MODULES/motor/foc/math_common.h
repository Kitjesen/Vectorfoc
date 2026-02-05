/**
 * @file    math_common.h
 * @brief   Common math utilities and constants for FOC.
 */

#ifndef FOC_MATH_COMMON_H
#define FOC_MATH_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Math constants */
#define MATH_PI (3.141592653589793f)
#define MATH_2PI (6.283185307179586f)
#define MATH_SQRT3 (1.732050807568877f)
#define MATH_ONE_BY_SQRT3 (0.577350269189626f)
#define MATH_SQRT3_BY_2 (0.866025403784439f)
#define MATH_2_BY_3 (0.666666666666667f)

/**
 * @brief  Normalize angle to [-π, π].
 * @param  angle [rad] Input angle.
 * @return [rad] Normalized angle.
 */
static inline float Math_NormalizeAngle(float angle) {
  while (angle > MATH_PI)
    angle -= MATH_2PI;
  while (angle < -MATH_PI)
    angle += MATH_2PI;
  return angle;
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
 * @return 1.0f (positive) or -1.0f (negative/zero).
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

#endif /* FOC_MATH_COMMON_H */
