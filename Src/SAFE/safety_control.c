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
 * @file safety_control.c
 * @brief safety
 * @version 4.0
 */
#include "safety_control.h"
#include "error_manager.h"
#include "error_types.h"
#include "hal_abstraction.h"
#include "platform.h"
static SafetyContext s_ctx = {0};
static void OnFaultDetected(uint32_t fault_bits, MOTOR_DATA *motor,
                            StateMachine *fsm);
static void ReportFaultToErrorManager(uint32_t fault_bits);
static inline void Safety_LatchFaultBits(uint32_t detected_faults,
                                         uint32_t new_faults);
static uint32_t Safety_TakePendingFaults(void);
static inline void Safety_ReportPendingFaults(MOTOR_DATA *motor,
                                              StateMachine *fsm);
static void Safety_AutoClearIfSafe(StateMachine *fsm);
void Safety_Init(const SafetyConfig *config) {
  if (config != NULL) {
    s_ctx.config = *config;
  } else {
    s_ctx.config = DEFAULT_SAFETY_CONFIG;
  }
  s_ctx.active_fault_bits = FAULT_NONE;
  s_ctx.pending_fault_bits = FAULT_NONE;
  s_ctx.fault_count = 0;
  s_ctx.initialized = true;
}
void Safety_Update(MOTOR_DATA *motor, StateMachine *fsm) {
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }
  // 1.
  uint32_t detected_faults = Detection_Check(motor);
  // 2. checkfault
  if (detected_faults != FAULT_NONE) {
    uint32_t active_snapshot;
    CRITICAL_SECTION_BEGIN();
    active_snapshot = s_ctx.active_fault_bits;
    CRITICAL_SECTION_END();
    uint32_t new_faults = detected_faults & ~active_snapshot;
    if (new_faults != FAULT_NONE) {
      s_ctx.last_fault_time = HAL_GetTick();
    }
    // update/fault
    Safety_LatchFaultBits(detected_faults, new_faults);
  } else {
    // 3. fault
    Safety_ReportPendingFaults(motor, fsm);
    if (s_ctx.active_fault_bits != FAULT_NONE &&
        s_ctx.config.auto_clear_on_recover) {
      // fault
      Safety_AutoClearIfSafe(fsm);
    }
    return;
  }
  Safety_ReportPendingFaults(motor, fsm);
}
/**
 * @brief safety (20kHz)
 * @note fault，1μs
 */
void Safety_Update_Fast(MOTOR_DATA *motor, StateMachine *fsm) {
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }
  //
  uint32_t detected_faults = Detection_Check_Fast(motor);
  // checkfault
  if (detected_faults != FAULT_NONE) {
    uint32_t active_snapshot;
    CRITICAL_SECTION_BEGIN();
    active_snapshot = s_ctx.active_fault_bits;
    CRITICAL_SECTION_END();
    uint32_t new_faults = detected_faults & ~active_snapshot;
    if (new_faults != FAULT_NONE) {
      s_ctx.last_fault_time = HAL_GetTick();
      // faultstate（），/
      if (fsm != NULL) {
        StateMachine_EnterFault(fsm, new_faults);
      }
    }
    Safety_LatchFaultBits(detected_faults, new_faults);
  }
}
/**
 * @brief safety (200Hz)
 * @note fault，3μs
 */
void Safety_Update_Slow(MOTOR_DATA *motor, StateMachine *fsm) {
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }
  //
  uint32_t detected_faults = Detection_Check_Slow(motor);
  // checkfault
  if (detected_faults != FAULT_NONE) {
    uint32_t active_snapshot;
    CRITICAL_SECTION_BEGIN();
    active_snapshot = s_ctx.active_fault_bits;
    CRITICAL_SECTION_END();
    uint32_t new_faults = detected_faults & ~active_snapshot;
    if (new_faults != FAULT_NONE) {
      s_ctx.last_fault_time = HAL_GetTick();
    }
    Safety_LatchFaultBits(detected_faults, new_faults);
  } else {
    // fault（）
    Safety_ReportPendingFaults(motor, fsm);
    if (s_ctx.active_fault_bits != FAULT_NONE &&
        s_ctx.config.auto_clear_on_recover) {
      Safety_AutoClearIfSafe(fsm);
    }
    return;
  }
  Safety_ReportPendingFaults(motor, fsm);
}
void Safety_ClearFaults(StateMachine *fsm) {
  Detection_Reset();
  CRITICAL_SECTION_BEGIN();
  s_ctx.active_fault_bits = FAULT_NONE;
  s_ctx.pending_fault_bits = FAULT_NONE;
  // [FIX] 重置故障计数器，避免历史数据影响
  s_ctx.fault_count = 0;
  CRITICAL_SECTION_END();
  // errorsafetyfault
  ErrorManager_ClearDomain(ERROR_DOMAIN_SAFETY);
  if (fsm != NULL) {
    CRITICAL_SECTION_BEGIN();
    StateMachine_ClearFault(fsm);
    CRITICAL_SECTION_END();
  }
}

void Safety_TriggerFault(uint32_t fault_bits, MOTOR_DATA *motor,
                         StateMachine *fsm) {
  (void)motor;
  if (fault_bits == FAULT_NONE) {
    return;
  }
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }

  uint32_t active_snapshot;
  CRITICAL_SECTION_BEGIN();
  active_snapshot = s_ctx.active_fault_bits;
  CRITICAL_SECTION_END();
  uint32_t new_faults = fault_bits & ~active_snapshot;
  if (new_faults != FAULT_NONE) {
    s_ctx.last_fault_time = HAL_GetTick();
    if (fsm != NULL) {
      StateMachine_EnterFault(fsm, new_faults);
    }
  }
  Safety_LatchFaultBits(fault_bits, new_faults);
}
bool Safety_HasActiveFault(void) {
  bool has_fault;
  CRITICAL_SECTION_BEGIN();
  has_fault = (s_ctx.active_fault_bits != FAULT_NONE);
  CRITICAL_SECTION_END();
  return has_fault;
}
uint32_t Safety_GetActiveFaultBits(void) {
  uint32_t bits;
  CRITICAL_SECTION_BEGIN();
  bits = s_ctx.active_fault_bits;
  CRITICAL_SECTION_END();
  return bits;
}
void Safety_RegisterFaultCallback(SafetyFaultCallback callback) {
  s_ctx.config.fault_callback = callback;
}
/* ==========  ========== */
static void OnFaultDetected(uint32_t fault_bits, MOTOR_DATA *motor,
                            StateMachine *fsm) {
  s_ctx.fault_count++;
  s_ctx.last_fault_time = HAL_GetTick(); // fault
  // 1. error
  ReportFaultToErrorManager(fault_bits);
  // 2. statefaultstate
  if (fsm != NULL) {
    StateMachine_EnterFault(fsm, fault_bits);
  }
  // 3. （CAN）
  if (s_ctx.config.fault_callback != NULL) {
    bool success = s_ctx.config.fault_callback(fault_bits, motor);
    if (!success) {
      // ，faultpending，wait
      Safety_LatchFaultBits(0, fault_bits);
    }
  }
}
static void ReportFaultToErrorManager(uint32_t fault_bits) {
  // faulterror
  if (fault_bits & FAULT_OVER_CURRENT) {
    ERROR_REPORT(ERROR_SAFETY_OVERCURRENT, "Over current detected");
  }
  if (fault_bits & FAULT_OVER_VOLTAGE) {
    ERROR_REPORT(ERROR_SAFETY_OVERVOLTAGE, "Over voltage detected");
  }
  if (fault_bits & FAULT_UNDER_VOLTAGE) {
    ERROR_REPORT(ERROR_SAFETY_UNDERVOLTAGE, "Under voltage detected");
  }
  if (fault_bits & FAULT_OVER_TEMP) {
    ERROR_REPORT(ERROR_SAFETY_OVERTEMP, "Over temperature detected");
  }
  if (fault_bits & FAULT_STALL_OVERLOAD) {
    ERROR_REPORT(ERROR_MOTOR_STALL, "Motor stall detected");
  }
  if (fault_bits & FAULT_ENCODER_LOSS) {
    ERROR_REPORT(ERROR_MOTOR_ENCODER_LOSS, "Encoder loss detected");
  }
  if (fault_bits & FAULT_CAN_TIMEOUT) {
    ERROR_REPORT(ERROR_COMM_TIMEOUT, "CAN communication timeout");
  }
  if (fault_bits & FAULT_CONTROL_INVALID) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE, "Invalid control configuration");
  }
  if (fault_bits & FAULT_ADC_STALE) {
    ERROR_REPORT(ERROR_HW_ADC_TIMEOUT, "ADC sample stale or incomplete");
  }
}
uint32_t Safety_GetLastFaultTime(void) { return s_ctx.last_fault_time; }
static inline void Safety_LatchFaultBits(uint32_t detected_faults,
                                         uint32_t new_faults) {
  CRITICAL_SECTION_BEGIN();
  s_ctx.active_fault_bits |= detected_faults;
  s_ctx.pending_fault_bits |= new_faults;
  CRITICAL_SECTION_END();
}
static uint32_t Safety_TakePendingFaults(void) {
  uint32_t pending;
  CRITICAL_SECTION_BEGIN();
  pending = s_ctx.pending_fault_bits;
  s_ctx.pending_fault_bits = FAULT_NONE;
  CRITICAL_SECTION_END();
  return pending;
}
static inline void Safety_ReportPendingFaults(MOTOR_DATA *motor,
                                              StateMachine *fsm) {
  uint32_t pending = Safety_TakePendingFaults();
  if (pending != FAULT_NONE) {
    OnFaultDetected(pending, motor, fsm);
  }
}
static void Safety_AutoClearIfSafe(StateMachine *fsm) {
  bool do_clear = false;
  CRITICAL_SECTION_BEGIN();
  if (s_ctx.active_fault_bits != FAULT_NONE &&
      s_ctx.pending_fault_bits == FAULT_NONE) {
    s_ctx.active_fault_bits = FAULT_NONE;
    s_ctx.pending_fault_bits = FAULT_NONE;
    do_clear = true;
  }
  CRITICAL_SECTION_END();
  if (!do_clear) {
    return;
  }
  Detection_Reset();
  ErrorManager_ClearDomain(ERROR_DOMAIN_SAFETY);
  if (fsm != NULL) {
    CRITICAL_SECTION_BEGIN();
    StateMachine_ClearFault(fsm);
    CRITICAL_SECTION_END();
  }
}
