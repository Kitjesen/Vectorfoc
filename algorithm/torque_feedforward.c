// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "torque_feedforward.h"
#include <stddef.h>

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
  float acceleration = 0.0f;
  if (state->has_last_velocity && dt_s > 1e-6f) {
    acceleration = (velocity_ref - state->last_velocity) / dt_s;
  }
  state->last_velocity = velocity_ref;
  state->has_last_velocity = true;
  return config->inertia * acceleration + config->friction * velocity_ref;
}
