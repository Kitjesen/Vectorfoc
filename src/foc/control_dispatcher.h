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

#ifndef CONTROL_DISPATCHER_H
#define CONTROL_DISPATCHER_H

#include "motor_runtime.h"

/* Initialize command rate limiters and the FOC state. */
void Control_Initialize(MOTOR_DATA *motor);

/* One current-control cycle: limit commands, select mode, and run the due
 * position/velocity loops before the inner current loop. OPEN and VF inject
 * voltage directly and skip the closed loops. */
void Control_RunCurrentCycle(MOTOR_DATA *motor);

/* Recompute current gains and controller limits after parameter changes. */
void Control_UpdateCurrentGains(MOTOR_DATA *motor);

/* Calibration/open-loop voltage injection, in volts and electrical radians. */
void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle);

#endif /* CONTROL_DISPATCHER_H */
