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

#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

#include "observer.h"

// SMO specific parameters and state
typedef struct {
  float alpha;
  float beta;
  // State variables
  float est_i_alpha;
  float est_i_beta;
  float est_bemf_alpha;
  float est_bemf_beta;
  float est_angle;
  float est_velocity;
  float pll_angle;
  float pll_velocity;
  float pll_kp;
  float pll_ki;
} SMO_Observer_t;

void SMO_Observer_Init(SMO_Observer_t *smo);
void SMO_Observer_Update(void *pMemory, MOTOR_DATA *motor);

#endif // SMO_OBSERVER_H
