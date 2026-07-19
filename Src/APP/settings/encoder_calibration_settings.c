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
#include "position_sensor.h"

#include <string.h>

typedef char EncoderCalibrationLutSizeMustMatch[
    PARAM_ENCODER_CALIBRATION_LUT_SIZE ==
            POSITION_SENSOR_CALIBRATION_LUT_SIZE
        ? 1
        : -1];

static void
EncoderCalibrationSettings_Apply(void *context,
                                 ParamEncoderCalibrationAction action,
                                 ParamEncoderCalibrationSnapshot *snapshot) {
  PositionSensorCalibrationSnapshot_t sensor_snapshot = {0};

  (void)context;

  switch (action) {
  case PARAM_ENCODER_CALIBRATION_CAPTURE:
    memset(snapshot, 0, sizeof(*snapshot));
    if (PositionSensor_CaptureCalibration(&sensor_snapshot) ==
        POSITION_SENSOR_STATUS_OK) {
      snapshot->valid = sensor_snapshot.valid;
      memcpy(snapshot->offset_lut, sensor_snapshot.offset_lut,
             sizeof(snapshot->offset_lut));
    }
    break;

  case PARAM_ENCODER_CALIBRATION_RESTORE:
    sensor_snapshot.valid = snapshot->valid;
    memcpy(sensor_snapshot.offset_lut, snapshot->offset_lut,
           sizeof(sensor_snapshot.offset_lut));
    (void)PositionSensor_RestoreCalibration(&sensor_snapshot);
    break;

  case PARAM_ENCODER_CALIBRATION_CLEAR:
    (void)PositionSensor_ClearCalibration();
    break;

  default:
    break;
  }
}

void EncoderCalibrationSettings_InstallAdapter(void) {
  ParamEncoderCalibration_SetAdapter(EncoderCalibrationSettings_Apply, NULL);
}
