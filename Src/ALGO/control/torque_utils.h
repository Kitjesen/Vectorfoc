// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CONTROL_TORQUE_UTILS_H
#define CONTROL_TORQUE_UTILS_H

#include <stdbool.h>

bool Control_TorqueToCurrent(float torque_nm, float torque_const_nm_per_a,
                             float *current_a);
float Control_InertiaTorque(float inertia_kg_m2,
                            float acceleration_turn_per_s2);
float Control_PositionVelocityLimit(float requested_limit_turn_per_s,
                                    float configured_limit_turn_per_s);

#endif /* CONTROL_TORQUE_UTILS_H */
