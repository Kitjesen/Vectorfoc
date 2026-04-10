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

#include "fault_def.h"

const DetectionConfig DEFAULT_DETECTION_CONFIG = {
    .over_voltage_threshold = FAULT_VBUS_OVERVOLT_V,
    .under_voltage_threshold = FAULT_VBUS_UNDERVOLT_V,
    .over_current_threshold = FAULT_OVER_CURRENT_A,
    .over_temp_threshold = FAULT_TEMP_ERROR_C,
    .temp_warning_threshold = FAULT_TEMP_WARN_C,
    .can_timeout_ms = FAULT_CAN_TIMEOUT_MS,
    .stall_detect_time_ms = FAULT_STALL_DETECT_TIME_MS,
    .stall_current_threshold = FAULT_STALL_CURRENT_A,
    .stall_velocity_threshold = FAULT_STALL_VELOCITY_RAD_S,
    .enable_voltage_protection = true,
    .enable_current_protection = true,
    .enable_temp_protection = true,
    .enable_stall_protection = true,
    .enable_can_timeout = false,
};

const SafetyConfig DEFAULT_SAFETY_CONFIG = {
    .detection_config =
        {
            .over_voltage_threshold = FAULT_VBUS_OVERVOLT_V,
            .under_voltage_threshold = FAULT_VBUS_UNDERVOLT_V,
            .over_current_threshold = FAULT_OVER_CURRENT_A,
            .over_temp_threshold = FAULT_TEMP_ERROR_C,
            .temp_warning_threshold = FAULT_TEMP_WARN_C,
            .can_timeout_ms = FAULT_CAN_TIMEOUT_MS,
            .stall_detect_time_ms = FAULT_STALL_DETECT_TIME_MS,
            .stall_current_threshold = FAULT_STALL_CURRENT_A,
            .stall_velocity_threshold = FAULT_STALL_VELOCITY_RAD_S,
            .enable_voltage_protection = true,
            .enable_current_protection = true,
            .enable_temp_protection = true,
            .enable_stall_protection = true,
            .enable_can_timeout = false,
        },
    .fault_callback = NULL,
    .auto_clear_on_recover = false,
};
