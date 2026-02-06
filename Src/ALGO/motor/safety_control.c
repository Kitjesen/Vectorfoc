/**
 * @file safety_control.c
 * @brief 安全控制层实现
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

  ErrorManager_Init(); // 确保错误管理器已初始化
}

void Safety_Update(MOTOR_DATA *motor, StateMachine *fsm) {
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }

  // 1. 执行物理量检测
  uint32_t detected_faults = Detection_Check(motor);

  // 2. 检查是否有新故障
  if (detected_faults != FAULT_NONE) {
    uint32_t active_snapshot;
    __disable_irq();
    active_snapshot = s_ctx.active_fault_bits;
    __enable_irq();

    uint32_t new_faults = detected_faults & ~active_snapshot;

    if (new_faults != FAULT_NONE) {
      s_ctx.last_fault_time = HAL_GetTick();
    }

    // 更新激活/待处理的故障
    Safety_LatchFaultBits(detected_faults, new_faults);
  } else {
    // 3. 故障恢复处理
    Safety_ReportPendingFaults(motor, fsm);
    if (s_ctx.active_fault_bits != FAULT_NONE &&
        s_ctx.config.auto_clear_on_recover) {
      // 自动清除已恢复的故障
      Safety_AutoClearIfSafe(fsm);
    }
    return;
  }

  Safety_ReportPendingFaults(motor, fsm);
}

/**
 * @brief 快速安全检测 (20kHz)
 * @note 仅检测过流等致命故障，执行时间约1μs
 */
void Safety_Update_Fast(MOTOR_DATA *motor, StateMachine *fsm) {
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }

  // 仅执行快速检测
  uint32_t detected_faults = Detection_Check_Fast(motor);

  // 检查是否有新故障
  if (detected_faults != FAULT_NONE) {
    uint32_t active_snapshot;
    __disable_irq();
    active_snapshot = s_ctx.active_fault_bits;
    __enable_irq();

    uint32_t new_faults = detected_faults & ~active_snapshot;

    if (new_faults != FAULT_NONE) {
      s_ctx.last_fault_time = HAL_GetTick();
      // 立即进入故障状态（快速停机），上报/日志延迟到慢速任务
      if (fsm != NULL) {
        StateMachine_EnterFault(fsm, new_faults);
      }
    }

    Safety_LatchFaultBits(detected_faults, new_faults);
  }
}

/**
 * @brief 慢速安全检测 (200Hz)
 * @note 检测缓变故障，执行时间约3μs
 */
void Safety_Update_Slow(MOTOR_DATA *motor, StateMachine *fsm) {
  if (!s_ctx.initialized) {
    Safety_Init(NULL);
  }

  // 执行慢速检测
  uint32_t detected_faults = Detection_Check_Slow(motor);

  // 检查是否有新故障
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
    // 故障恢复处理（仅在慢速检测中执行）
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
  __enable_irq();

  // 清除错误管理器中的安全域故障
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

/* ========== 内部函数实现 ========== */

static void OnFaultDetected(uint32_t fault_bits, MOTOR_DATA *motor,
                            StateMachine *fsm) {
  s_ctx.fault_count++;
  s_ctx.last_fault_time = HAL_GetTick(); // 记录故障时间戳

  // 1. 上报到统一错误管理器
  ReportFaultToErrorManager(fault_bits);

  // 2. 触发状态机故障状态
  if (fsm != NULL) {
    StateMachine_EnterFault(fsm, fault_bits);
  }

  // 3. 调用用户回调（用于发送CAN帧等）
  if (s_ctx.config.fault_callback != NULL) {
    bool success = s_ctx.config.fault_callback(fault_bits, motor);
    if (!success) {
      // 发送失败，将故障位加回pending，等待下一次循环重试
      Safety_LatchFaultBits(0, fault_bits);
    }
  }
}

static void ReportFaultToErrorManager(uint32_t fault_bits) {
  // 直接报告故障位到错误管理器
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
