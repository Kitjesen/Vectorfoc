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

#include "param_encoder_calibration_internal.h"

#include <string.h>

static ParamEncoderCalibrationAdapter s_adapter = NULL;
static void *s_adapter_context = NULL;

void ParamEncoderCalibration_SetAdapter(ParamEncoderCalibrationAdapter adapter,
                                        void *context) {
  s_adapter_context = context;
  s_adapter = adapter;
}

static void
ParamEncoderCalibration_Notify(ParamEncoderCalibrationAction action,
                               ParamEncoderCalibrationSnapshot *snapshot) {
  if (s_adapter != NULL) {
    s_adapter(s_adapter_context, action, snapshot);
  }
}

void ParamEncoderCalibration_Collect(FlashParamData *flash_data) {
  ParamEncoderCalibrationSnapshot snapshot = {0};

  if (flash_data == NULL) {
    return;
  }

  ParamEncoderCalibration_Notify(PARAM_ENCODER_CALIBRATION_CAPTURE, &snapshot);
  flash_data->encoder_calib_valid = snapshot.valid ? 1U : 0U;
  memset(flash_data->encoder_calib_reserved, 0,
         sizeof(flash_data->encoder_calib_reserved));
  memcpy(flash_data->encoder_offset_lut, snapshot.offset_lut,
         sizeof(flash_data->encoder_offset_lut));
}

void ParamEncoderCalibration_Restore(const FlashParamData *flash_data) {
  ParamEncoderCalibrationSnapshot snapshot = {0};

  if (flash_data == NULL) {
    return;
  }

  snapshot.valid = flash_data->encoder_calib_valid == 1U;
  memcpy(snapshot.offset_lut, flash_data->encoder_offset_lut,
         sizeof(snapshot.offset_lut));
  ParamEncoderCalibration_Notify(PARAM_ENCODER_CALIBRATION_RESTORE, &snapshot);
}

void ParamEncoderCalibration_Clear(void) {
  ParamEncoderCalibrationSnapshot snapshot = {0};
  ParamEncoderCalibration_Notify(PARAM_ENCODER_CALIBRATION_CLEAR, &snapshot);
}

bool ParamEncoderCalibration_IsFlashDataValid(
    const FlashParamData *flash_data) {
  if (flash_data == NULL || flash_data->encoder_calib_valid > 1U) {
    return false;
  }

  for (uint32_t i = 0U; i < sizeof(flash_data->encoder_calib_reserved); ++i) {
    if (flash_data->encoder_calib_reserved[i] != 0U) {
      return false;
    }
  }
  return true;
}
