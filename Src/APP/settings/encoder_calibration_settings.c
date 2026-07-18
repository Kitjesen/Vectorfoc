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

#include "encoder_calibration_settings.h"

#include "param_encoder_calibration.h"

#if !defined(TEST_ENV)
#include "config.h"
#if defined(BOARD_XSTAR)
#include "abz_encoder.h"
#include "hall_encoder.h"
#else
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#include "tmr3109_encoder.h"
#else
#include "mt6816_encoder.h"
#endif
#endif
#include <string.h>

static void
EncoderCalibrationSettings_Apply(void *context,
                                 ParamEncoderCalibrationAction action,
                                 ParamEncoderCalibrationSnapshot *snapshot) {
  (void)context;

  switch (action) {
  case PARAM_ENCODER_CALIBRATION_CAPTURE:
#if defined(BOARD_XSTAR)
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    snapshot->valid = hall_data.calib_valid;
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
    snapshot->valid = abz_data.calib_valid;
#endif
#else
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
    snapshot->valid = tmr3109_encoder_data.calib_valid;
    memcpy(snapshot->offset_lut, tmr3109_encoder_data.offset_lut,
           sizeof(snapshot->offset_lut));
#else
    snapshot->valid = encoder_data.calib_valid;
    memcpy(snapshot->offset_lut, encoder_data.offset_lut,
           sizeof(snapshot->offset_lut));
#endif
#endif
    break;

  case PARAM_ENCODER_CALIBRATION_RESTORE:
#if defined(BOARD_XSTAR)
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    hall_data.calib_valid = snapshot->valid;
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
    abz_data.calib_valid = snapshot->valid;
#endif
#else
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
    tmr3109_encoder_data.calib_valid = snapshot->valid;
    if (snapshot->valid) {
      memcpy(tmr3109_encoder_data.offset_lut, snapshot->offset_lut,
             sizeof(tmr3109_encoder_data.offset_lut));
    }
#else
    encoder_data.calib_valid = snapshot->valid;
    if (snapshot->valid) {
      memcpy(encoder_data.offset_lut, snapshot->offset_lut,
             sizeof(encoder_data.offset_lut));
    }
#endif
#endif
    break;

  case PARAM_ENCODER_CALIBRATION_CLEAR:
#if defined(BOARD_XSTAR)
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    hall_data.calib_valid = false;
    hall_data.offset_rad = 0.0f;
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
    abz_data.calib_valid = false;
    abz_data.offset_counts = 0;
    abz_data.offset_rad = 0.0f;
#endif
#else
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
    tmr3109_encoder_data.calib_valid = false;
    tmr3109_encoder_data.offset_counts = 0;
    memset(tmr3109_encoder_data.offset_lut, 0,
           sizeof(tmr3109_encoder_data.offset_lut));
#else
    encoder_data.calib_valid = false;
    encoder_data.is_calibrated = false;
    encoder_data.offset_counts = 0;
    memset(encoder_data.offset_lut, 0, sizeof(encoder_data.offset_lut));
#endif
#endif
    break;

  default:
    break;
  }
}
#endif

void EncoderCalibrationSettings_InstallAdapter(void) {
#if !defined(TEST_ENV)
  ParamEncoderCalibration_SetAdapter(EncoderCalibrationSettings_Apply, NULL);
#else
  ParamEncoderCalibration_SetAdapter(NULL, NULL);
#endif
}
