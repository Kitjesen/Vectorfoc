#ifndef COMMON_H
#define COMMON_H
#include "main.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef unsigned char bool_t; //
#ifndef M_PI
#define M_PI (3.14159265358979323846f) // M_PI
#endif
#define M_2PI (6.28318530717958647692f)    // 2
#define M_3PI_2 (4.71238898038469f)        // 3/2
#define _SQRT3 (1.7320508075688772935f)    // 3
#define _SQRT3_2 (0.86602540378443864f)    // 3
#define ONE_BY_SQRT2 (0.7071067811865475f) // 12
#define ONE_BY_SQRT3 (0.57735026919f)      // 13
#define TWO_BY_SQRT3 (1.15470053838f)      // 23
#define SQ(x) ((x) * (x))                               //
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))            //
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0f)) // angle
#define RAD2DEG_f(rad) ((rad) * (float)(180.0f / M_PI)) // angle
#define RPM2RADPS_f(rpm)                                                       \
  ((rpm) * (float)((2.0f * M_PI) / 60.0f)) // speedspeed/velocity
#define RADPS2RPM_f(rad_per_sec)                                               \
  ((rad_per_sec) * (float)(60.0f / (2.0f * M_PI))) // speed/velocityspeed
#define ABS(a) ((a > 0.0f) ? (a) : (-a))                   //
#define min(a, b) (((a) < (b)) ? (a) : (b))                //
#define max(a, b) (((a) > (b)) ? (a) : (b))                //
#define CLAMP(x, lower, upper) (min(upper, max(x, lower))) //
#define SIGN(x) (((x) < 0.0f) ? -1.0f : 1.0f)              //
#define UTILS_LP_FAST(value, sample, filter_constant)                          \
  (value -= (filter_constant) * (value - (sample)))
#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N)                           \
  UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))
static inline int mod(int dividend, int divisor) {
  int r = dividend % divisor;
  return (r < 0) ? (r + divisor) : r;
}
static inline void int_to_data(int val, uint8_t *data) {
  data[0] = *(((uint8_t *)(&val)) + 0);
  data[1] = *(((uint8_t *)(&val)) + 1);
  data[2] = *(((uint8_t *)(&val)) + 2);
  data[3] = *(((uint8_t *)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data) {
  int tmp_int;
  *(((uint8_t *)(&tmp_int)) + 0) = data[0];
  *(((uint8_t *)(&tmp_int)) + 1) = data[1];
  *(((uint8_t *)(&tmp_int)) + 2) = data[2];
  *(((uint8_t *)(&tmp_int)) + 3) = data[3];
  return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data) {
  data[0] = *(((uint8_t *)(&val)) + 0);
  data[1] = *(((uint8_t *)(&val)) + 1);
  data[2] = *(((uint8_t *)(&val)) + 2);
  data[3] = *(((uint8_t *)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data) {
  float tmp_float;
  *(((uint8_t *)(&tmp_float)) + 0) = data[0];
  *(((uint8_t *)(&tmp_float)) + 1) = data[1];
  *(((uint8_t *)(&tmp_float)) + 2) = data[2];
  *(((uint8_t *)(&tmp_float)) + 3) = data[3];
  return tmp_float;
}
static inline void float4_to_data7(float val, uint8_t *data) {
  data[4] = *(((uint8_t *)(&val)) + 0);
  data[5] = *(((uint8_t *)(&val)) + 1);
  data[6] = *(((uint8_t *)(&val)) + 2);
  data[7] = *(((uint8_t *)(&val)) + 3);
}
static inline uint32_t cpu_enter_critical(void) { return 0; }
static inline void cpu_exit_critical(uint32_t priority_mask) {
  (void)priority_mask;
}
static inline float fmodf_pos(float x, float y) {
  float out = fmodf(x, y);
  if (out < 0.0f) {
    out += y;
  }
  return out;
}
static inline float wrap_pm(float x, float pm_range) {
  return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}
static inline float wrap_pm_pi(float theta) { return wrap_pm(theta, M_PI); }
#endif // !COMMON_H