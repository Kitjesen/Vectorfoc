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

#ifndef CALIB_ENCODER_H
#define CALIB_ENCODER_H

#include "calibration_context.h"
#include "motor.h"
#include "config.h"

/**
 * @file calib_encoder.h
 * @brief Encoder, pole pair, and direction calibration module
 */

/**
 * @brief Update direction and pole pair identification
 */
CalibResult DirectionPoleCalib_Update(MOTOR_DATA *motor,
                                      DirectionPoleCalibContext *ctx);

/**
 * @brief Update encoder offset calibration
 */
CalibResult EncoderCalib_Update(MOTOR_DATA *motor, EncoderCalibContext *ctx);

#endif // CALIB_ENCODER_H
