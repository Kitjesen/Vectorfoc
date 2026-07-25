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

/**
 * @file param_encoder_calibration.h
 * @brief Persistence seam for board-specific encoder calibration state.
 */
#ifndef PARAM_ENCODER_CALIBRATION_H
#define PARAM_ENCODER_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define PARAM_ENCODER_CALIBRATION_LUT_SIZE 128U

/** Portable representation of the encoder data that belongs in Flash. */
typedef struct {
  bool valid;
  int16_t offset_lut[PARAM_ENCODER_CALIBRATION_LUT_SIZE];
} ParamEncoderCalibrationSnapshot;

/** Direction of data flow across the encoder-calibration seam. */
typedef enum {
  PARAM_ENCODER_CALIBRATION_CAPTURE = 0,
  PARAM_ENCODER_CALIBRATION_RESTORE,
  PARAM_ENCODER_CALIBRATION_CLEAR,
} ParamEncoderCalibrationAction;

/**
 * Board adapter for the encoder calibration persistence seam.
 *
 * `snapshot` is non-NULL. Capture fills it, restore consumes it, and clear
 * resets the board-specific runtime calibration state.
 */
typedef void (*ParamEncoderCalibrationAdapter)(
    void *context, ParamEncoderCalibrationAction action,
    ParamEncoderCalibrationSnapshot *snapshot);

/** Install or clear the board adapter. Pass NULL to clear it. */
void ParamEncoderCalibration_SetAdapter(ParamEncoderCalibrationAdapter adapter,
                                        void *context);

#endif /* PARAM_ENCODER_CALIBRATION_H */
