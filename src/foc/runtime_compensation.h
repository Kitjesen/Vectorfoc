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

#ifndef CONTROL_COMPENSATION_H
#define CONTROL_COMPENSATION_H

#include "motor_runtime.h"
#include "algorithm/field_weakening.h"

/* Runtime boundary for pure feedforward and field-weakening algorithms. */
void Feedforward_Update(MOTOR_DATA *motor);
void Feedforward_Reset(void);
float Feedforward_GetCurrent(const MOTOR_DATA *motor);
float FieldWeakening_Calculate(const MOTOR_DATA *motor,
                               const FieldWeakening_Config_t *config, float dt_s);
void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *config);
void FieldWeakening_Reset(void);

#endif /* CONTROL_COMPENSATION_H */
