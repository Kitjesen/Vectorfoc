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
