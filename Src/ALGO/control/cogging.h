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

#ifndef COGGING_H
#define COGGING_H
#include "motor.h"
void CoggingComp_Update(MOTOR_DATA *motor);
float CoggingComp_GetCurrent(const MOTOR_DATA *motor);
void CoggingComp_StartCalibration(MOTOR_DATA *motor);
void CoggingComp_StopCalibration(MOTOR_DATA *motor);
bool CoggingComp_IsCalibrating(void);
bool CoggingComp_IsValid(void);
/**
 * @brief getcalibration
 * @return  (0 ~ COGGING_MAP_SIZE-1)，calibration 0
 */
uint16_t CoggingComp_GetCalibStep(void);
#endif // COGGING_H
