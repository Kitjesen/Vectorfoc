// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef ALGORITHM_FIELD_WEAKENING_H
#define ALGORITHM_FIELD_WEAKENING_H

#include <stdbool.h>

typedef struct {
  float max_weakening_current;
  float start_velocity;
} FieldWeakening_Config_t;

typedef struct {
  float id_integral;
} FieldWeakening_State_t;

void FieldWeakening_AlgorithmReset(FieldWeakening_State_t *state);
float FieldWeakening_AlgorithmUpdate(FieldWeakening_State_t *state,
                                      const FieldWeakening_Config_t *config,
                                      float velocity_turn_s,
                                      bool voltage_saturated,
                                      float dt_s);

#endif /* ALGORITHM_FIELD_WEAKENING_H */
