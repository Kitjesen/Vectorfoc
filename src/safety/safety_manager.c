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
 * @file safety_manager.c
 * @brief safety
 * @version 4.0
 */
#include "safety_manager.h"
#include "error_manager.h"
#include "error_types.h"
#include "hardware_interface.h"
#include "current_calibration.h"
#include "pwm_interface.h"
static SafetyContext s_ctx = {0};
static void OnFaultDetected(uint32_t fault_bits, MOTOR_DATA *motor,
                            StateMachine *fsm);
static void ReportFaultToErrorManager(uint32_t fault_bits);
static inline void Safety_LatchFaultBits(uint32_t detected_faults,
                                         uint32_t new_faults);
static uint32_t Safety_TakePendingFaults(void);
static inline void Safety_ReportPendingFaults(MOTOR_DATA *motor,
                                              StateMachine *fsm);
static void Safety_EnterFaultIfNeeded(uint32_t fault_bits, StateMachine *fsm);
static void Safety_LatchPendingReportBits(uint32_t fault_bits);
static uint32_t Safety_TakePendingReportBits(void);
static void Safety_RetryPendingReports(MOTOR_DATA *motor);
static void Safety_AutoClearIfSafe(StateMachine *fsm);
void Safety_Init(const SafetyConfig *config) {
  if (config != NULL) {
    s_ctx.config = *config;
  } else {
    s_ctx.config = DEFAULT_SAFETY_CONFIG;
  }
  s_ctx.active_fault_bits = FAULT_NONE;
  s_ctx.pending_fault_bits = FAULT_NONE;
  s_ctx.pending_report_bits = FAULT_NONE;
  s_ctx.fsm_reported_fault_bits = FAULT_NONE;
  s_ctx.fault_count = 0;
  s_ctx.initialized = true;
  ErrorManager_Init(); // errorinit
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
    uint32_t irq_state = HAL_EnterCritical();
    active_snapshot = s_ctx.active_fault_bits;
    HAL_ExitCritical(irq_state);
    uint32_t new_faults = detected_faults & ~active_snapshot;
    if (new_faults != FAULT_NONE) {
      s_ctx.last_fault_time = HAL_GetTick();
      // faultstate（），/
      if (fsm != NULL) {
        Safety_EnterFaultIfNeeded(new_faults, fsm);
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
  Safety_RetryPendingReports(motor);
  uint32_t detected_faults = Detection_Check_Slow(motor);
  // checkfault
  if (detected_faults != FAULT_NONE) {
    uint32_t active_snapshot;
    uint32_t irq_state = HAL_EnterCritical();
    active_snapshot = s_ctx.active_fault_bits;
    HAL_ExitCritical(irq_state);
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
bool Safety_CanEnable(MOTOR_DATA *motor) {
  uint32_t irq_state = HAL_EnterCritical();
  bool allowed = motor && CurrentCalib_IsReady() &&
                 s_ctx.active_fault_bits == FAULT_NONE &&
                 Detection_PreviewFaults(motor) == FAULT_NONE;
  HAL_ExitCritical(irq_state);
  return allowed;
}
bool Safety_ClearFaults(StateMachine *fsm) {
  /* A reset is a stopped state, never permission to replay an old enable. */
  MHAL_PWM_Disable();
  /* Preserve a fast-path fault in history/callback delivery before clearing
   * the active latch.  A failed callback remains queued for the slow task. */
  Safety_ReportPendingFaults(NULL, NULL);
  uint32_t irq_state = HAL_EnterCritical();
  Detection_Reset();
  uint32_t remaining = Detection_PreviewFaults(&motor_data);
  if (remaining != FAULT_NONE || !CurrentCalib_IsReady()) {
    s_ctx.active_fault_bits |= remaining;
    s_ctx.pending_fault_bits |= remaining;
    if (fsm && remaining) StateMachine_EnterFault(fsm, remaining);
    HAL_ExitCritical(irq_state);
    return false;
  }
  s_ctx.active_fault_bits = FAULT_NONE;
  s_ctx.pending_fault_bits = FAULT_NONE;
  s_ctx.fsm_reported_fault_bits = FAULT_NONE;
  s_ctx.fault_count = 0;
  if (fsm) {
    if (!StateMachine_ClearFault(fsm))
      StateMachine_RequestState(fsm, STATE_SWITCH_ON_DISABLED);
  }
  ErrorManager_ClearDomain(ERROR_DOMAIN_SAFETY);
  HAL_ExitCritical(irq_state);
  return true;
}
void Safety_TriggerFault(uint32_t fault_bits, MOTOR_DATA *motor,
                         StateMachine *fsm) {
  (void)motor;
  if (fault_bits == FAULT_NONE) return;
  if (!s_ctx.initialized) Safety_Init(NULL);

  uint32_t irq_state = HAL_EnterCritical();
  uint32_t new_faults = fault_bits & ~s_ctx.active_fault_bits;
  HAL_ExitCritical(irq_state);
  if (new_faults != FAULT_NONE) {
    s_ctx.last_fault_time = HAL_GetTick();
    Safety_EnterFaultIfNeeded(new_faults, fsm);
  }
  Safety_LatchFaultBits(fault_bits, new_faults);
}
bool Safety_HasActiveFault(void) {
  bool has_fault;
  uint32_t irq_state = HAL_EnterCritical();
  has_fault = (s_ctx.active_fault_bits != FAULT_NONE);
  HAL_ExitCritical(irq_state);
  return has_fault;
}
uint32_t Safety_GetActiveFaultBits(void) {
  uint32_t bits;
  uint32_t irq_state = HAL_EnterCritical();
  bits = s_ctx.active_fault_bits;
  HAL_ExitCritical(irq_state);
  return bits;
}
void Safety_RegisterFaultCallback(SafetyFaultCallback callback) {
  uint32_t irq_state = HAL_EnterCritical();
  s_ctx.config.fault_callback = callback;
  HAL_ExitCritical(irq_state);
}
static SafetyFaultCallback Safety_GetFaultCallback(void) {
  uint32_t irq_state = HAL_EnterCritical();
  SafetyFaultCallback callback = s_ctx.config.fault_callback;
  HAL_ExitCritical(irq_state);
  return callback;
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
    Safety_EnterFaultIfNeeded(fault_bits, fsm);
  }
  // 3. （CAN）
  SafetyFaultCallback callback = Safety_GetFaultCallback();
  if (callback != NULL) {
    bool success = callback(fault_bits, motor);
    if (!success) {
      Safety_LatchPendingReportBits(fault_bits);
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
  uint32_t irq_state = HAL_EnterCritical();
  s_ctx.active_fault_bits |= detected_faults;
  s_ctx.pending_fault_bits |= new_faults;
  HAL_ExitCritical(irq_state);
}
static uint32_t Safety_TakePendingFaults(void) {
  uint32_t pending;
  uint32_t irq_state = HAL_EnterCritical();
  pending = s_ctx.pending_fault_bits;
  s_ctx.pending_fault_bits = FAULT_NONE;
  HAL_ExitCritical(irq_state);
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
  uint32_t irq_state = HAL_EnterCritical();
  bool can_clear = s_ctx.active_fault_bits != FAULT_NONE &&
                   s_ctx.pending_fault_bits == FAULT_NONE &&
                   s_ctx.pending_report_bits == FAULT_NONE;
  HAL_ExitCritical(irq_state);
  if (can_clear) (void)Safety_ClearFaults(fsm);
}

static void Safety_EnterFaultIfNeeded(uint32_t fault_bits, StateMachine *fsm) {
  if (fsm == NULL || fault_bits == FAULT_NONE) return;
  uint32_t irq_state = HAL_EnterCritical();
  uint32_t unreported = fault_bits & ~s_ctx.fsm_reported_fault_bits;
  s_ctx.fsm_reported_fault_bits |= unreported;
  HAL_ExitCritical(irq_state);
  if (unreported != FAULT_NONE) StateMachine_EnterFault(fsm, unreported);
}

static void Safety_LatchPendingReportBits(uint32_t fault_bits) {
  uint32_t irq_state = HAL_EnterCritical();
  s_ctx.pending_report_bits |= fault_bits;
  HAL_ExitCritical(irq_state);
}

static uint32_t Safety_TakePendingReportBits(void) {
  uint32_t irq_state = HAL_EnterCritical();
  uint32_t pending = s_ctx.pending_report_bits;
  s_ctx.pending_report_bits = FAULT_NONE;
  HAL_ExitCritical(irq_state);
  return pending;
}

static void Safety_RetryPendingReports(MOTOR_DATA *motor) {
  SafetyFaultCallback callback = Safety_GetFaultCallback();
  if (callback == NULL) return;
  uint32_t pending = Safety_TakePendingReportBits();
  if (pending != FAULT_NONE && !callback(pending, motor))
    Safety_LatchPendingReportBits(pending);
}
