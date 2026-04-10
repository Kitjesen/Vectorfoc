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

#ifndef FIELD_WEAKENING_H
#define FIELD_WEAKENING_H

#include "motor.h"

typedef struct {
  float max_weakening_current;
  float start_velocity;
} FieldWeakening_Config_t;

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *cfg);

#endif // FIELD_WEAKENING_H
