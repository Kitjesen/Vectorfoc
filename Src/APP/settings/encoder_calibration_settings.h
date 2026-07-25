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
 * @file encoder_calibration_settings.h
 * @brief APP adapter for selected-sensor calibration persistence.
 */
#ifndef ENCODER_CALIBRATION_SETTINGS_H
#define ENCODER_CALIBRATION_SETTINGS_H

/**
 * Install before Param_SystemInitOnce(). The selected PositionSensor must be
 * initialized before Param_SystemInitOnce() invokes the restore adapter.
 */
void EncoderCalibrationSettings_InstallAdapter(void);

#endif /* ENCODER_CALIBRATION_SETTINGS_H */
