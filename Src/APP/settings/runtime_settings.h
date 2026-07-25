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
 * @file runtime_settings.h
 * @brief APP adapter that applies persisted parameter values to live runtime
 *        state after the relevant hardware and communication modules exist.
 */
#ifndef RUNTIME_SETTINGS_H
#define RUNTIME_SETTINGS_H

/**
 * Install the production adapter at the parameter runtime seam.
 *
 * Call this only after the motor runtime, encoder and protocol modules are
 * initialized.  The caller then requests the initial bulk apply through
 * Param_ApplyRuntimeState().
 */
void RuntimeSettings_InstallAdapter(void);

/**
 * Apply the persisted encoder offset during boot, after encoder initialization
 * and before the first feedback capture.
 *
 * @return the underlying encoder HAL result.
 */
int RuntimeSettings_ApplyEncoderOffset(void);

#endif /* RUNTIME_SETTINGS_H */
