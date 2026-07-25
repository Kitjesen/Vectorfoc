#include "encoder_angle_math.h"

#include <math.h>
#include <stdio.h>

#define CHECK_NEAR(actual, expected, tolerance)                                \
  do {                                                                         \
    if (fabsf((actual) - (expected)) > (tolerance)) {                          \
      printf("FAIL %s:%d: %s=%f expected=%f\n", __FILE__, __LINE__, #actual, \
             (double)(actual), (double)(expected));                            \
      return 1;                                                                \
    }                                                                          \
  } while (0)

int main(void) {
  const float two_pi = 6.28318530717959f;
  EncoderAngleResult_t without_offset;
  EncoderAngleResult_t with_offset;

  EncoderAngleMath_Compute(4096, 0.25f, 0, 16384u, 7u, &without_offset);
  EncoderAngleMath_Compute(4096, 0.25f, 1024, 16384u, 7u, &with_offset);

  CHECK_NEAR(with_offset.mechanical_angle_rad,
             without_offset.mechanical_angle_rad, 1.0e-6f);
  CHECK_NEAR(without_offset.mechanical_angle_rad,
             (4096.25f / 16384.0f) * two_pi, 1.0e-6f);
  if (fabsf(with_offset.electrical_angle_rad -
            without_offset.electrical_angle_rad) < 0.1f) {
    puts("FAIL electrical offset did not change commutation angle");
    return 1;
  }

  EncoderAngleMath_Compute(100, 0.0f, 200, 16384u, 7u, &with_offset);
  if (!(with_offset.mechanical_angle_rad >= 0.0f &&
        with_offset.mechanical_angle_rad < two_pi &&
        with_offset.electrical_angle_rad >= -3.1415928f &&
        with_offset.electrical_angle_rad <= 3.1415928f)) {
    puts("FAIL angle wrapping contract");
    return 1;
  }

  puts("Encoder angle math tests PASSED");
  return 0;
}
