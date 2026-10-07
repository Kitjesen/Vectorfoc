// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "field_weakening.h"
#include <math.h>
#include <stddef.h>

static float clamp(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

void FieldWeakening_AlgorithmReset(FieldWeakening_State_t *state) {
  if (state != NULL) state->id_integral = 0.0f;
}

float FieldWeakening_AlgorithmUpdate(FieldWeakening_State_t *state,
                                      const FieldWeakening_Config_t *config,
                                      float velocity_turn_s,
                                      bool voltage_saturated,
                                      float dt_s) {
  if (state == NULL || config == NULL) return 0.0f;
  if (!isfinite(config->max_weakening_current) || !isfinite(config->start_velocity) ||
      !isfinite(velocity_turn_s) || config->max_weakening_current <= 0.0f) {
    FieldWeakening_AlgorithmReset(state);
    return 0.0f;
  }
  if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.1f) return 0.0f;
  if (dt_s > 0.0f && dt_s <= 0.1f) {
    if (voltage_saturated) {
      state->id_integral -= 100.0f * dt_s;
    } else if (state->id_integral < 0.0f) {
      state->id_integral += 10.0f * dt_s;
      if (state->id_integral > 0.0f) state->id_integral = 0.0f;
    }
  }
  state->id_integral = clamp(state->id_integral,
                             -config->max_weakening_current, 0.0f);

  float abs_velocity = fabsf(velocity_turn_s);
  float linear = 0.0f;
  if (config->start_velocity > 0.0f && abs_velocity > config->start_velocity) {
    float ratio = (abs_velocity - config->start_velocity) /
                  config->start_velocity;
    linear = -config->max_weakening_current * clamp(ratio, 0.0f, 1.0f);
  }
  return linear < state->id_integral ? linear : state->id_integral;
}
