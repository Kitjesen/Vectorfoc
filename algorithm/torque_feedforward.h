// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef ALGORITHM_FEEDFORWARD_H
#define ALGORITHM_FEEDFORWARD_H

#include <stdbool.h>

typedef struct {
  float inertia;
  float friction;
} Feedforward_Config_t;

typedef struct {
  float last_velocity;
  bool has_last_velocity;
} Feedforward_State_t;

void Feedforward_AlgorithmReset(Feedforward_State_t *state);
float Feedforward_AlgorithmUpdate(Feedforward_State_t *state,
                                   const Feedforward_Config_t *config,
                                   float velocity_ref,
                                   float dt_s);

bool Control_TorqueToCurrent(float torque_nm, float torque_const_nm_per_a,
                             float *current_a);
float Control_InertiaTorque(float inertia_kg_m2, float acceleration_turn_per_s2);
float Control_PositionVelocityLimit(float requested_limit, float configured_limit);

#endif /* ALGORITHM_FEEDFORWARD_H */
