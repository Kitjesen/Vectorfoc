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

#ifndef FLUX_CALIB_H
#define FLUX_CALIB_H

#include "calibration_context.h"
#include "motor.h"

/**
 * @file flux_calib.h
 * @brief Flux calibration module
 *
 * Measures motor flux using Back-EMF method
 */

/**
 * @brief Initialize flux calibration
 */
CalibResult FluxCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Update flux calibration
 */
CalibResult FluxCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Finish flux calibration
 */
CalibResult FluxCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx);

#endif // FLUX_CALIB_H
