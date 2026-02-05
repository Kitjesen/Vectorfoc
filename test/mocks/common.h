#ifndef COMMON_H
#define COMMON_H

// Sanitized Common Header for Host Testing
// Removed: #include "main.h"
// Removed: #include "stm32g4xx_hal.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned char bool_t;

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif
#define M_2PI (6.28318530717958647692f)
#define M_3PI_2 (4.71238898038469f)

#define SQ(x) ((x) * (x))
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0f))
#define RAD2DEG_f(rad) ((rad) * (float)(180.0f / M_PI))
#define RPM2RADPS_f(rpm) ((rpm) * (float)((2.0f * M_PI) / 60.0f))
#define RADPS2RPM_f(rad_per_sec)                                               \
  ((rad_per_sec) * (float)(60.0f / (2.0f * M_PI)))

#define ABS(a) ((a > 0.0f) ? (a) : (-a))
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, lower, upper) (min(upper, max(x, lower)))
#define SIGN(x) (((x) < 0.0f) ? -1.0f : 1.0f)

// Utilities
static inline int mod(int dividend, int divisor) {
  int r = dividend % divisor;
  return (r < 0) ? (r + divisor) : r;
}

// Data conversion helpers skipped if not used by Motor Core directly
// or can be kept if standard C

static inline uint32_t cpu_enter_critical(void) { return 0; }
static inline void cpu_exit_critical(uint32_t priority_mask) {
  (void)priority_mask;
}

static inline float fmodf_pos(float x, float y) {
  float out = fmodf(x, y);
  if (out < 0.0f)
    out += y;
  return out;
}

static inline float wrap_pm(float x, float pm_range) {
  return fmodf_pos(x + pm_range, 2.0f * pm_range) - pm_range;
}
static inline float wrap_pm_pi(float theta) { return wrap_pm(theta, M_PI); }

#endif // COMMON_H
