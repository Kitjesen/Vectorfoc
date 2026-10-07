// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef ENCODER_ANGLE_MATH_H
#define ENCODER_ANGLE_MATH_H

#include <stdint.h>

typedef struct {
  float mechanical_angle_rad;
  float electrical_angle_rad;
} EncoderAngleResult_t;

static inline float EncoderAngleMath_WrapPositive(float angle,
                                                   float period) {
  while (angle >= period) {
    angle -= period;
  }
  while (angle < 0.0f) {
    angle += period;
  }
  return angle;
}

static inline float EncoderAngleMath_WrapSigned(float angle, float period) {
  const float half_period = 0.5f * period;
  while (angle > half_period) {
    angle -= period;
  }
  while (angle <= -half_period) {
    angle += period;
  }
  return angle;
}

/**
 * Compute mechanically absolute and electrically calibrated angles.
 *
 * Electrical-zero calibration must never alter the wrapped mechanical angle.
 * `count_in_cpr` is the direction-corrected native count.
 */
static inline void EncoderAngleMath_Compute(int32_t count_in_cpr,
                                            float interpolation,
                                            int32_t electrical_offset_counts,
                                            uint32_t counts_per_revolution,
                                            uint8_t pole_pairs,
                                            EncoderAngleResult_t *result) {
  const float two_pi = 6.28318530717959f;
  if (result == 0 || counts_per_revolution == 0u || pole_pairs == 0u) {
    if (result != 0) {
      result->mechanical_angle_rad = 0.0f;
      result->electrical_angle_rad = 0.0f;
    }
    return;
  }

  const float radians_per_count = two_pi / (float)counts_per_revolution;
  const float mechanical_counts = (float)count_in_cpr + interpolation;
  const float electrical_counts =
      mechanical_counts - (float)electrical_offset_counts;

  result->mechanical_angle_rad = EncoderAngleMath_WrapPositive(
      mechanical_counts * radians_per_count, two_pi);
  result->electrical_angle_rad = EncoderAngleMath_WrapSigned(
      electrical_counts * radians_per_count * (float)pole_pairs, two_pi);
}

#endif /* ENCODER_ANGLE_MATH_H */
