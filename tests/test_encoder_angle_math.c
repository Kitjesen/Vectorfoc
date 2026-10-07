#include "encoder_angle_math.h"

#include <assert.h>
#include <math.h>

static void electrical_offset_does_not_change_mechanical_angle(void) {
  EncoderAngleResult_t without_offset;
  EncoderAngleResult_t with_offset;

  EncoderAngleMath_Compute(4096, 0.25f, 0, 16384u, 7u, &without_offset);
  EncoderAngleMath_Compute(4096, 0.25f, 1024, 16384u, 7u, &with_offset);

  assert(fabsf(with_offset.mechanical_angle_rad -
               without_offset.mechanical_angle_rad) < 1.0e-6f);
  assert(fabsf(with_offset.electrical_angle_rad -
               without_offset.electrical_angle_rad) > 0.1f);
}

static void output_angles_are_wrapped(void) {
  EncoderAngleResult_t result;
  EncoderAngleMath_Compute(100, 0.0f, 200, 16384u, 7u, &result);

  assert(result.mechanical_angle_rad >= 0.0f);
  assert(result.mechanical_angle_rad < 6.2831854f);
  assert(result.electrical_angle_rad >= -3.1415928f);
  assert(result.electrical_angle_rad <= 3.1415928f);
}

int main(void) {
  electrical_offset_does_not_change_mechanical_angle();
  output_angles_are_wrapped();
  return 0;
}
