// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include "motor.h"

// Feedforward Parameters
typedef struct {
  float inertia;
  float friction_coeff;
  // Add other FF parameters if needed
} Feedforward_Params_t;

void Feedforward_Init(Feedforward_Params_t *params);
void Feedforward_Update(MOTOR_DATA *motor, const Feedforward_Params_t *params);

#endif // FEEDFORWARD_H
