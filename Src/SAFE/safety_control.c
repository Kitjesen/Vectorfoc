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
  ErrorManager_Init(); // errorinit
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
    __disable_irq();
    active_snapshot = s_ctx.active_fault_bits;
    __enable_irq();
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
    __disable_irq();
    active_snapshot = s_ctx.active_fault_bits;
    __enable_irq();
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
    __disable_irq();
    active_snapshot = s_ctx.active_fault_bits;
    __enable_irq();
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
  __disable_irq();
  s_ctx.active_fault_bits = FAULT_NONE;
  s_ctx.pending_fault_bits = FAULT_NONE;
  // [FIX] 重置故障计数器，避免历史数据影响
  s_ctx.fault_count = 0;
  __enable_irq();
  // errorsafetyfault
  ErrorManager_ClearDomain(ERROR_DOMAIN_SAFETY);
  if (fsm != NULL) {
    __disable_irq();
    StateMachine_ClearFault(fsm);
    __enable_irq();
  }
}
bool Safety_HasActiveFault(void) {
  bool has_fault;
  __disable_irq();
  has_fault = (s_ctx.active_fault_bits != FAULT_NONE);
  __enable_irq();
  return has_fault;
}
uint32_t Safety_GetActiveFaultBits(void) {
  uint32_t bits;
  __disable_irq();
  bits = s_ctx.active_fault_bits;
  __enable_irq();
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
}
uint32_t Safety_GetLastFaultTime(void) { return s_ctx.last_fault_time; }
static inline void Safety_LatchFaultBits(uint32_t detected_faults,
                                         uint32_t new_faults) {
  __disable_irq();
  s_ctx.active_fault_bits |= detected_faults;
  s_ctx.pending_fault_bits |= new_faults;
  __enable_irq();
}
static uint32_t Safety_TakePendingFaults(void) {
  uint32_t pending;
  __disable_irq();
  pending = s_ctx.pending_fault_bits;
  s_ctx.pending_fault_bits = FAULT_NONE;
  __enable_irq();
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
  __disable_irq();
  if (s_ctx.active_fault_bits != FAULT_NONE &&
      s_ctx.pending_fault_bits == FAULT_NONE) {
    s_ctx.active_fault_bits = FAULT_NONE;
    s_ctx.pending_fault_bits = FAULT_NONE;
    do_clear = true;
  }
  __enable_irq();
  if (!do_clear) {
    return;
  }
  Detection_Reset();
  ErrorManager_ClearDomain(ERROR_DOMAIN_SAFETY);
  if (fsm != NULL) {
    __disable_irq();
    StateMachine_ClearFault(fsm);
    __enable_irq();
  }
}
