#include "motor.h"
#include "bsp_dwt.h"
#include "current_calib.h"
#include "flux_calib.h"
#include "led.h"
#include "config.h"
#include "control/control.h"
#include "hal_pwm.h" // For MHAL_PWM_Brake (统一 HAL)
#include "mt6816_encoder.h"
#include "param_access.h"
#include "param_table.h"
#include "pid.h"

#include "rsls_calib.h"
#include "safety_control.h"
#include <stdlib.h>

extern Motor_HAL_Handle_t g431_hal_handle;

/* 定义全局DS402状态机实例 */
extern StateMachine g_ds402_state_machine;
extern uint8_t g_can_id;

/**
 * @brief 全局电机数据结构
 * @note 电池配置: 14串锂电池 (14S Li-ion)
 *       - 标称电压: 14 × 3.7V = 51.8V
 *       - 充满电压: 14 × 4.2V = 58.8V
 *       - 截止电压: 14 × 3.0V = 42.0V
 */
extern MOTOR_DATA motor_data;

/**
 * @brief 校准状态管理任务
 * 依次执行: 电流零偏 -> 电阻/电感 -> 磁链校准
 */
static void MotorInitializeTask(MOTOR_DATA *motor) {
  CalibResult result;

  switch (motor->state.Sub_State) {
  case CURRENT_CALIBRATING:
    result = CurrentCalib_Update(motor, &motor->calib_ctx);
    if (result == CALIB_SUCCESS) {
      Init_Motor_Calib(motor); // 自动进入 RSLS 校准
    } else if (result != CALIB_IN_PROGRESS) {
      motor->state.State_Mode = STATE_MODE_GUARD;
    }
    break;

  case RSLS_CALIBRATING:
    result = RSLSCalib_Update(motor, &motor->calib_ctx, CURRENT_MEASURE_PERIOD);
    if (result == CALIB_SUCCESS) {
      motor->state.Sub_State = FLUX_CALIBRATING;
    } else if (result != CALIB_IN_PROGRESS) {
      motor->state.State_Mode = STATE_MODE_GUARD;
    }
    break;

  case FLUX_CALIBRATING:
    result = FluxCalib_Update(motor, &motor->calib_ctx);
    if (result == CALIB_SUCCESS) {
      motor->state.Sub_State = SUB_STATE_IDLE;
      motor->state.State_Mode = STATE_MODE_RUNNING;
      Param_ScheduleSave(); // 校准完成，延迟保存（ISR安全）
    } else if (result != CALIB_IN_PROGRESS) {
      motor->state.State_Mode = STATE_MODE_GUARD; // 校准失败
    }
    break;

  default:
    break; // SUB_STATE_IDLE 等不需要处理
  }
}

/**
 * @brief FOC 主状态机
 * 调度 IDLE -> DETECTING -> RUNNING -> GUARD 状态切换
 */
void MotorStateTask(MOTOR_DATA *motor) {
  // 1. 同步高层 FSM 到低层状态
  MotorState fsm_state = StateMachine_GetState(&g_ds402_state_machine);
  // 使用静态变量记录上一次状态, 实现状态切换检测
  static MotorState last_state = STATE_NOT_READY_TO_SWITCH_ON;

  // 2. 状态映射 (兼容层)
  switch (fsm_state) {
  case STATE_OPERATION_ENABLED:
    motor->state.State_Mode = STATE_MODE_RUNNING;
    break;
  case STATE_CALIBRATING:
    motor->state.State_Mode = STATE_MODE_DETECTING;
    break;
  case STATE_FAULT:
  case STATE_FAULT_REACTION_ACTIVE:
    motor->state.State_Mode = STATE_MODE_GUARD;
    break;
  default:
    motor->state.State_Mode = STATE_MODE_IDLE;
    break;
  }

  // 3. 状态切换动作 (Entry Action)
  if (fsm_state != last_state) {
    // 当进入 IDLE 或 GUARD 状态时，执行一次性清理
    if (motor->state.State_Mode == STATE_MODE_IDLE ||
        motor->state.State_Mode == STATE_MODE_GUARD) {
      PID_clear(&motor->IqPID);
      PID_clear(&motor->IdPID);
      PID_clear(&motor->VelPID);
      PID_clear(&motor->PosPID);
      FOC_Algorithm_ResetState(&motor->algo_state);
      MHAL_PWM_Brake();
    }
    last_state = fsm_state;
  }

  // 4. 持续性动作 (Do Action)
  switch (motor->state.State_Mode) {
  case STATE_MODE_RUNNING: // 运行模式
    MotorControl_Run(motor);
    break;

  case STATE_MODE_DETECTING: // 校准/检测模式
    MotorInitializeTask(motor);
    if (motor->state.Sub_State == SUB_STATE_IDLE) {
      // Exit to Switch On Disabled
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
    // Check if calibration failed (Transitioned to GUARD by
    // MotorInitializeTask)
    else if (motor->state.State_Mode == STATE_MODE_GUARD) {
      // 同步故障状态到 FSM
      StateMachine_EnterFault(&g_ds402_state_machine, FAULT_STALL_OVERLOAD);
    }
    break;

  case STATE_MODE_IDLE:
  case STATE_MODE_GUARD:
    // 不需要持续执行任何动作 (已在 Entry 阶段处理了停止和清零)
    break;

  default:
    break;
  }
}

/**
 * @brief 安全保护任务 (200Hz)
 * 检测过压/欠压/过流/过温/超速，并驱动 LED 指示
 */
void MotorGuardTask(MOTOR_DATA *motor) {
  // 1. 运行慢速安全检测 (200Hz)
  Safety_Update_Slow(motor, &g_ds402_state_machine);

  // 2. 获取检测结果
  uint32_t fault_bits = Safety_GetActiveFaultBits();

  // 3. LED 状态指示
  if (motor->state.State_Mode == STATE_MODE_IDLE ||
      motor->state.State_Mode == STATE_MODE_DETECTING) {
    RGB_DisplayColorById(9); // 上电/校准中
  } else if (Safety_HasActiveFault()) {
    // 处于保护状态
    RGB_DisplayColorById(0); //  故障保护状态

    // 确保电机状态同步切换到 GUARD
    if (motor->state.State_Mode != STATE_MODE_GUARD) {
      motor->state.State_Mode = STATE_MODE_GUARD;
      // 映射第一个故障位到 Fault_State（已废弃，仅用于兼容）
      // @deprecated 建议使用 Safety_GetActiveFaultBits() 获取完整故障位掩码
      if (fault_bits & FAULT_OVER_VOLTAGE)
        motor->state.Fault_State = FAULT_STATE_OVER_VOLTAGE;
      else if (fault_bits & FAULT_UNDER_VOLTAGE)
        motor->state.Fault_State = FAULT_STATE_UNDER_VOLTAGE;
      else if (fault_bits & FAULT_OVER_CURRENT)
        motor->state.Fault_State = FAULT_STATE_OVER_CURRENT;
      else if (fault_bits & FAULT_OVER_TEMP)
        motor->state.Fault_State = FAULT_STATE_OVER_TEMPERATURE;
      else if (fault_bits & FAULT_STALL_OVERLOAD)
        motor->state.Fault_State = FAULT_STATE_SPEEDING;
      else if (fault_bits & FAULT_ENCODER_LOSS)
        motor->state.Fault_State = FAULT_STATE_ENCODER_LOSS;
    }
  } else if (motor->state.State_Mode == STATE_MODE_RUNNING) {
    RGB_DisplayColorById(3); //  正常运行
  }
}
