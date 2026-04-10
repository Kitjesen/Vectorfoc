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

#ifndef MOTOR_PLANT_H
#define MOTOR_PLANT_H

#include <stdbool.h>

typedef struct {
  // Motor Parameters (Model)
  float R;    // Phase Res [Ohm]
  float L;    // Phase Inductance [H]
  float Flux; // Flux Linkage [Wb]
  float J;    // Inertia [kg m^2]
  float B;    // Friction [Nm s/rad]
  int P;      // Pole Pairs

  // State Variables
  float i_alpha; // Alpha current [A]
  float i_beta;  // Beta current [A]
  float omega;   // Mechanical Velocity [rad/s]
  float theta;   // Mechanical Angle [rad]

  // Current simulation step inputs (Back EMF)
  float bemf_alpha;
  float bemf_beta;

  float dt; // Simulation timestep [s]

} MotorPlant_t;

void MotorPlant_Init(MotorPlant_t *plant);
void MotorPlant_Step(MotorPlant_t *plant, float v_alpha, float v_beta,
                     float load_torque);
void MotorPlant_GetCurrents(MotorPlant_t *plant, float *ia, float *ib,
                            float *ic);

#endif
