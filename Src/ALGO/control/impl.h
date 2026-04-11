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

#ifndef CONTROL_MODES_IMPL_H
#define CONTROL_MODES_IMPL_H
#include "context.h"
#include "motor.h"
// Mode Implementations
void ControlImpl_Open(MOTOR_DATA *motor);
void ControlImpl_Torque(MOTOR_DATA *motor, MotorControlCtx *ctx);
void ControlImpl_Velocity(MOTOR_DATA *motor);
void ControlImpl_Position(MOTOR_DATA *motor);
void ControlImpl_VelocityRamp(MOTOR_DATA *motor);
void ControlImpl_PositionRamp(MOTOR_DATA *motor, MotorControlCtx *ctx);
void ControlImpl_MIT(MOTOR_DATA *motor);
void ControlImpl_VF(MOTOR_DATA *motor);
void ControlImpl_IF(MOTOR_DATA *motor);
// Helpers
void ControlImpl_SetThetaFromEncoder(MOTOR_DATA *motor);
void ControlImpl_SetPidLimits(MOTOR_DATA *motor);
/**
 * @brief voltage (open loop)
 * @note calibrationopen loop，PIDoutputvoltage。
 * @param motor motor
 * @param Vd Daxisvoltage [V]
 * @param Vq Qaxisvoltage [V]
 * @param angle angle [rad]
 */
void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle);
#endif // CONTROL_MODES_IMPL_H
