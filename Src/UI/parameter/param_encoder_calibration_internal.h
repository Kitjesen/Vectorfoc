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

/** Internal Flash mapping for the encoder-calibration persistence module. */
#ifndef PARAM_ENCODER_CALIBRATION_INTERNAL_H
#define PARAM_ENCODER_CALIBRATION_INTERNAL_H

#include "param_encoder_calibration.h"
#include "param_storage.h"

void ParamEncoderCalibration_Collect(FlashParamData *flash_data);
void ParamEncoderCalibration_Restore(const FlashParamData *flash_data);
void ParamEncoderCalibration_Clear(void);
bool ParamEncoderCalibration_IsFlashDataValid(const FlashParamData *flash_data);

#endif /* PARAM_ENCODER_CALIBRATION_INTERNAL_H */
