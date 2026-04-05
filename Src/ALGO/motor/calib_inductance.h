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

#ifndef CALIB_INDUCTANCE_H
#define CALIB_INDUCTANCE_H

#include "calibration_context.h"
#include "motor.h"
#include "config.h"

/**
 * @file calib_inductance.h
 * @brief Inductance calibration module
 */

/**
 * @brief Update inductance calibration state machine
 *
 * @param motor Motor data
 * @param ctx   Inductance calibration context
 * @return CalibResult Calibration result
 */
CalibResult InductanceCalib_Update(MOTOR_DATA *motor,
                                   InductanceCalibContext *ctx, float dt);

#endif // CALIB_INDUCTANCE_H
