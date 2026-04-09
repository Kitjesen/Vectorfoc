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

#ifndef OBSERVER_H
#define OBSERVER_H

#include "motor.h"

/**
 * @brief Observer generic interface
 */
typedef struct {
  void (*update)(void *pMemory, MOTOR_DATA *motor);
  void (*reset)(void *pMemory);
} Observer_Interface_t;

// Function prototypes for integration
void Observer_Init(MOTOR_DATA *motor);
void Observer_Update(MOTOR_DATA *motor);

#endif // OBSERVER_H
