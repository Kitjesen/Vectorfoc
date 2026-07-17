// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "torque_utils.h"
#include "common.h"
#include <math.h>
#include <stddef.h>

bool Control_TorqueToCurrent(float torque_nm, float torque_const_nm_per_a,
                             float *current_a) {
  if (current_a == NULL)
    return false;
  *current_a = 0.0f;
  if (!isfinite(torque_nm) || !isfinite(torque_const_nm_per_a) ||
      fabsf(torque_const_nm_per_a) < 1e-6f) {
    return false;
  }
  *current_a = torque_nm / torque_const_nm_per_a;
  return isfinite(*current_a);
}

float Control_InertiaTorque(float inertia_kg_m2,
                            float acceleration_turn_per_s2) {
  if (!isfinite(inertia_kg_m2) || !isfinite(acceleration_turn_per_s2))
    return 0.0f;
  return inertia_kg_m2 * acceleration_turn_per_s2 * M_2PI;
}

float Control_PositionVelocityLimit(float requested_limit_turn_per_s,
                                    float configured_limit_turn_per_s) {
  if (!isfinite(requested_limit_turn_per_s) ||
      !isfinite(configured_limit_turn_per_s)) {
    return 0.0f;
  }
  return fminf(fabsf(requested_limit_turn_per_s),
               fabsf(configured_limit_turn_per_s));
}
