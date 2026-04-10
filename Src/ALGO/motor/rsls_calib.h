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

#ifndef RSLS_CALIB_H
#define RSLS_CALIB_H

#include "calibration_context.h"
#include "motor.h"


/**
 * @file rsls_calib.h
 * @brief Resistance, Inductance, Direction, Pole Pair, and Encoder Calibration Module
 *
 * This module coordinates the complete automatic motor parameter identification flow.
 */

/**
 * @brief Initialize Rs/Ls calibration
 */
CalibResult RSLSCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Update Rs/Ls calibration state machine
 */
CalibResult RSLSCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx,
                             float dt);

/**
 * @brief Get current calibration progress percentage
 */
uint8_t RSLSCalib_GetProgress(CalibrationContext *ctx);

#endif // RSLS_CALIB_H
