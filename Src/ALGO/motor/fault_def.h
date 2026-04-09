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

#ifndef FAULT_DEF_H
#define FAULT_DEF_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* fault */
typedef enum {
  FAULT_NONE = 0,
  FAULT_OVER_TEMP = (1 << 0),            // bit0: motorfault (>145℃)
  FAULT_DRIVER_CHIP = (1 << 1),          // bit1: driverfault
  FAULT_UNDER_VOLTAGE = (1 << 2),        // bit2: fault (<12V)
  FAULT_OVER_VOLTAGE = (1 << 3),         // bit3: fault (>60V)
  FAULT_CURRENT_B = (1 << 4),            // bit4: Bphasecurrentsample
  FAULT_CURRENT_C = (1 << 5),            // bit5: Cphasecurrentsample
  FAULT_ENCODER_LOSS = (1 << 6),         // bit6: encoder/fault
  FAULT_ENCODER_UNCALIBRATED = (1 << 7), // bit7: encoder
  FAULT_HARDWARE_ID = (1 << 8),          // bit8: fault
  FAULT_POSITION_INIT = (1 << 9),        // bit9: positioninitfault
  FAULT_STALL_OVERLOAD = (1 << 14),      // bit14: motoroverloadprotection
  FAULT_CURRENT_A = (1 << 16),           // bit16: Aphasecurrentsample
} FaultBit;
#define FAULT_OVER_CURRENT (FAULT_CURRENT_A | FAULT_CURRENT_B | FAULT_CURRENT_C)
/* warning */
typedef enum {
  WARNING_NONE = 0,
  WARNING_OVER_TEMP = (1 << 0), // bit0: motor
} WarningBit;
/* fault */
typedef struct {
  uint32_t fault_code;      // fault
  uint32_t warning_code;    // warning
  uint32_t fault_count;     // fault
  uint32_t last_fault_time; // fault
} FaultInfo;
/* state -  */
typedef struct {
  float vbus_filtered;   // filtervoltage
  float current_peak;    // current
  float temp_filtered;   // filtertemperature
  float temp_calculated; // calctemperature
  uint32_t stall_counter; //
  bool is_stall;          //
  uint32_t last_can_time; // CAN
  bool is_can_timeout;    // CANtimeout
  uint32_t encoder_err_consecutive; // encodererror
  uint32_t encoder_err_count;       // encodererror()
} DetectionState;
/* protectionconfigparam */
typedef struct {
  /* voltageprotectionthreshold */
  float over_voltage_threshold;  // threshold
  float under_voltage_threshold; // threshold
  /* currentprotectionthreshold */
  float over_current_threshold; // threshold
  /* temperatureprotectionthreshold */
  float over_temp_threshold;    // threshold
  float temp_warning_threshold; // threshold
  /* CANtimeout */
  uint32_t can_timeout_ms; // CANtimeout
  /*  */
  uint32_t stall_detect_time_ms;  //
  float stall_current_threshold;  // currentthreshold
  float stall_velocity_threshold; // speed/velocitythreshold
  /* enable */
  bool enable_voltage_protection;
  bool enable_current_protection;
  bool enable_temp_protection;
  bool enable_stall_protection;
  bool enable_can_timeout;
} DetectionConfig;
/* fault */
typedef bool (*SafetyFaultCallback)(uint32_t fault_code, void *motor);
/* safetyconfig */
typedef struct {
  DetectionConfig detection_config;   /**< config */
  SafetyFaultCallback fault_callback; /**< fault（） */
  bool auto_clear_on_recover;         /**< fault */
} SafetyConfig;
/* safety */
typedef struct {
  SafetyConfig config;
  uint32_t active_fault_bits;  /**< FaultBit */
  uint32_t pending_fault_bits; /**< /fault */
  uint32_t fault_count;        /**< fault */
  uint32_t last_fault_time;    /**< fault (ms) */
  bool initialized;
} SafetyContext;
/* protectionparam */
#define FAULT_VBUS_OVERVOLT_V 60.0f
#define FAULT_VBUS_UNDERVOLT_V 12.0f
#define FAULT_OVER_CURRENT_A 90.0f
#define FAULT_TEMP_ERROR_C 145.0f
#define FAULT_TEMP_WARN_C 130.0f
#define FAULT_CAN_TIMEOUT_MS 0
#define FAULT_STALL_DETECT_TIME_MS 500
#define FAULT_STALL_CURRENT_A 80.0f
#define FAULT_STALL_VELOCITY_RAD_S 0.1f
/* encoderfaultthreshold */
#define FAULT_ENCODER_ERR_CONSECUTIVE_MAX 20U
#define FAULT_ENCODER_ERR_COUNT_MAX 20U
/* fault */
#define FAULT_FILTER_ALPHA_SLOW 0.99f // voltage/temperaturefilter
#define FAULT_FILTER_ALPHA_FAST 0.01f // voltage/temperaturefilter
#define STALL_DETECT_COUNT_PER_MS 20  //  (20kHzfrequency)
/* /safetyconfig ( fault_def.c) */
extern const DetectionConfig DEFAULT_DETECTION_CONFIG;
extern const SafetyConfig DEFAULT_SAFETY_CONFIG;
#endif /* FAULT_DEF_H */
