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

#include "motor.h"
#include "bsp_dwt.h"
#include "current_calib.h"
#include "flux_calib.h"
/* led.h 已从 motor.c 中移除（ALGO→UI 反向依赖）；
 * LED 状态更新由 task_guard.c 负责，读取 motor->state 后调用 led.h。 */
#include "config.h"
#include "control/control.h"
#include "hal_pwm.h" // For MHAL_PWM_Brake ( HAL)
#include "param_access.h"
#include "param_table.h"
#include "pid.h"
#include "rsls_calib.h"
#include "safety_control.h"
#include <stdlib.h>
extern Motor_HAL_Handle_t HW_MOTOR_HAL_HANDLE;
/* DS402state */
extern StateMachine g_ds402_state_machine;
extern uint8_t g_can_id;
/**
 * @brief motor
 * @note config: 14 (14S Li-ion)
 *       - voltage: 14 × 3.7V = 51.8V
 *       - voltage: 14 × 4.2V = 58.8V
 *       - voltage: 14 × 3.0V = 42.0V
 */
extern MOTOR_DATA motor_data;
/**
 * @brief calibrationstate
 * : current -> / -> fluxcalibration
 */
static void MotorInitializeTask(MOTOR_DATA *motor) {
  CalibResult result;
  switch (motor->state.Sub_State) {
  case CURRENT_CALIBRATING:
    result = CurrentCalib_Update(motor, &motor->calib_ctx);
    if (result == CALIB_SUCCESS) {
      motor->last_calib_result = CALIB_SUCCESS;
      if (motor->calib_type_requested == 3) {
        // Current-only calibration: done, return to idle
        motor->state.Sub_State = SUB_STATE_IDLE;
        motor->state.State_Mode = STATE_MODE_RUNNING;
        Param_ScheduleSave();
      } else {
        Init_Motor_Calib(motor); //  RSLS calibration
      }
    } else if (result != CALIB_IN_PROGRESS) {
      motor->last_calib_result = result;
      motor->state.State_Mode = STATE_MODE_GUARD;
    }
    break;
  case RSLS_CALIBRATING:
    result = RSLSCalib_Update(motor, &motor->calib_ctx, CURRENT_MEASURE_PERIOD);
    if (result == CALIB_SUCCESS) {
      motor->last_calib_result = CALIB_SUCCESS;
      motor->state.Sub_State = FLUX_CALIBRATING;
    } else if (result != CALIB_IN_PROGRESS) {
      motor->last_calib_result = result;
      motor->state.State_Mode = STATE_MODE_GUARD;
    }
    break;
  case FLUX_CALIBRATING:
    result = FluxCalib_Update(motor, &motor->calib_ctx);
    if (result == CALIB_SUCCESS) {
      motor->last_calib_result = CALIB_SUCCESS;
      motor->state.Sub_State = SUB_STATE_IDLE;
      motor->state.State_Mode = STATE_MODE_RUNNING;
      Param_ScheduleSave(); // calibrationdone，（ISRsafety）
    } else if (result != CALIB_IN_PROGRESS) {
      motor->last_calib_result = result;
      motor->state.State_Mode = STATE_MODE_GUARD; // calibration
    }
    break;
  default:
    break; // SUB_STATE_IDLE
  }
}
/**
 * @brief FOC state
 *  IDLE -> DETECTING -> RUNNING -> GUARD state
 */
void MotorStateTask(MOTOR_DATA *motor) {
  // 1.  FSM state
  MotorState fsm_state = StateMachine_GetState(&g_ds402_state_machine);
  // state, state
  static MotorState last_state = STATE_NOT_READY_TO_SWITCH_ON;
  // 2. state ()
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
  // 3. state (Entry Action)
  if (fsm_state != last_state) {
    //  IDLE  GUARD state，
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
  // 4.  (Do Action)
  switch (motor->state.State_Mode) {
  case STATE_MODE_RUNNING: // runningmode
    MotorControl_Run(motor);
    break;
  case STATE_MODE_DETECTING: // calibration/mode
    MotorInitializeTask(motor);
    if (motor->state.Sub_State == SUB_STATE_IDLE) {
      // Exit to Switch On Disabled
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
    // Check if calibration failed (Transitioned to GUARD by
    // MotorInitializeTask)
    else if (motor->state.State_Mode == STATE_MODE_GUARD) {
      // faultstate FSM
      StateMachine_EnterFault(&g_ds402_state_machine, FAULT_STALL_OVERLOAD);
    }
    break;
  case STATE_MODE_IDLE:
  case STATE_MODE_GUARD:
    //  ( Entry stop)
    break;
  default:
    break;
  }
}
/**
 * @brief 安全监控任务（200Hz）— 仅安全检测，不含 LED 输出
 *
 * LED 状态由 task_guard.c 根据 motor->state 更新，
 * 避免 ALGO 层直接依赖 UI 层 led.h（消除 ALGO→UI 反向依赖）。
 */
void MotorGuardTask(MOTOR_DATA *motor) {
  // 1. 执行慢速安全检测（200Hz）
  Safety_Update_Slow(motor, &g_ds402_state_machine);
  // 2. 若有故障，更新 motor state 到 GUARD
  if (Safety_HasActiveFault()) {
    if (motor->state.State_Mode != STATE_MODE_GUARD) {
      motor->state.State_Mode = STATE_MODE_GUARD;
      uint32_t fault_bits = Safety_GetActiveFaultBits();
      /* 更新已弃用的 Fault_State 字段（兼容旧代码，正式应用 Safety_GetActiveFaultBits()） */
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
  }
  /* LED 状态反映由 task_guard.c 的 Motor_UpdateLED() 完成 */
}
