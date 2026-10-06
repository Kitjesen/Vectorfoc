#ifndef CONTROL_TEST_ENCODER_H
#define CONTROL_TEST_ENCODER_H

#include "common.h"

/* Reuse the host handle definitions, but replace the shared mock's identity
 * normalize_angle with the arithmetic used by both production boards. */
#define normalize_angle ControlTest_IdentityAngleUnused
#include "../mocks/mt6816_encoder.h"
#undef normalize_angle

static inline float normalize_angle(float angle) {
  float a = fmodf(angle, M_2PI);
  return a >= 0 ? a : (a + M_2PI);
}

#endif
