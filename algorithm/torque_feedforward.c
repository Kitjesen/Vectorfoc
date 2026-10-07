// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "torque_feedforward.h"
#include <stddef.h>
#include <math.h>
#include "foc_transforms.h"

void Feedforward_AlgorithmReset(Feedforward_State_t *state) {
  if (state == NULL) return;
  state->last_velocity = 0.0f;
  state->has_last_velocity = false;
}

float Feedforward_AlgorithmUpdate(Feedforward_State_t *state,
                                   const Feedforward_Config_t *config,
                                   float velocity_ref,
                                   float dt_s) {
  if (state == NULL || config == NULL) return 0.0f;
  if (!isfinite(config->inertia) || !isfinite(config->friction) ||
      !isfinite(velocity_ref) || !isfinite(dt_s) || dt_s <= 0.0f) {
    Feedforward_AlgorithmReset(state);
    return 0.0f;
  }
  float acceleration = 0.0f;
  if (state->has_last_velocity && dt_s > 1e-6f) {
    acceleration = (velocity_ref - state->last_velocity) / dt_s;
  }
  state->last_velocity = velocity_ref;
  state->has_last_velocity = true;
  return Control_InertiaTorque(config->inertia, acceleration) + config->friction * velocity_ref;
}

bool Control_TorqueToCurrent(float torque_nm, float torque_const_nm_per_a,
                             float *current_a) {
  if (current_a == NULL) return false;
  *current_a = 0.0f;
  if (!isfinite(torque_nm) || !isfinite(torque_const_nm_per_a) ||
      fabsf(torque_const_nm_per_a) < 1e-6f) return false;
  *current_a = torque_nm / torque_const_nm_per_a;
  return isfinite(*current_a);
}
float Control_InertiaTorque(float inertia_kg_m2, float acceleration_turn_per_s2) {
  if (!isfinite(inertia_kg_m2) || !isfinite(acceleration_turn_per_s2)) return 0.0f;
  return inertia_kg_m2 * acceleration_turn_per_s2 * MATH_2PI;
}
float Control_PositionVelocityLimit(float requested_limit, float configured_limit) {
  if (!isfinite(requested_limit) || !isfinite(configured_limit)) return 0.0f;
  return fminf(fabsf(requested_limit), fabsf(configured_limit));
}
