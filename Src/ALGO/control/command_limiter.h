// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CONTROL_COMMAND_LIMITER_H
#define CONTROL_COMMAND_LIMITER_H

#include "context.h"

void ControlCommandLimiter_Init(MotorControlCtx *context,
                                const MOTOR_DATA *motor);
void ControlCommandLimiter_Reset(MotorControlCtx *context,
                                 const MOTOR_DATA *motor);
void ControlCommandLimiter_Update(MotorControlCtx *context,
                                  const MOTOR_DATA *motor, float dt);

#endif /* CONTROL_COMMAND_LIMITER_H */
