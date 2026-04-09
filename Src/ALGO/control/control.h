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

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "motor.h"
// Forward declaration if motor.h inclusion is guarded
struct MOTOR_DATA_s;
// typedef struct MOTOR_DATA_s MOTOR_DATA; // Removed to avoid redefinition
/**
 * @brief  Initialize Motor Control Context
 * @param  motor Motor Control Data
 */
void Control_Init(MOTOR_DATA *motor);
/**
 * @brief Motor Control Main Dispatch Entry
 *
 * Coordinates the various parts of the control system:
 * 1. Rate Limiting - Process input commands
 * 2. Mode Dispatching - Delegate to modes/impl
 * 3. Outer Loop - Velocity/Position Control (loops/outer)
 * 4. Inner Loop - FOC Current Control (loops/inner)
 *
 * @param motor Motor Control Data
 */
void MotorControl_Run(MOTOR_DATA *motor);
/**
 * @brief set PID
 */
void SetPIDLimit(MOTOR_DATA *motor, float current_max_out,
                 float current_max_iout, float vel_max_out, float vel_max_iout,
                 float pos_limit);
/**
 * @brief updatecurrentparam (gain)
 */
void CurrentLoop_UpdateGain(MOTOR_DATA *motor);
#endif // MOTOR_CONTROL_H
