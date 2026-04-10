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

#ifndef CURRENT_CALIB_H
#define CURRENT_CALIB_H

#include "calibration_context.h"
#include "motor.h"

/**
 * @file current_calib.h
 * @brief Current offset calibration module
 *
 * Responsible for acquiring current sensor zero offsets
 */

/**
 * @brief Initialize current offset calibration
 *
 * @param motor Motor data structure
 * @param ctx Calibration context
 * @return CalibResult Calibration result status
 */
CalibResult CurrentCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Update current offset calibration (called every control cycle)
 *
 * @param motor Motor data structure
 * @param ctx Calibration context
 * @return CalibResult Calibration result status
 */
CalibResult CurrentCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Finish current offset calibration and apply offsets
 *
 * @param motor Motor data structure
 * @param ctx Calibration context
 * @return CalibResult Calibration result status
 */
CalibResult CurrentCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx);

#endif // CURRENT_CALIB_H
