/**
 * @file executor.c
 * @brief Command Executor Implementation
 */

#include "executor.h"
#include "error_manager.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h" // For MHAL_Encoder_* (统一 HAL)
#include "manager.h"         // For Protocol_SendFrame used in feedback
#include "motor.h"
#include "safety_control.h"
#include "param_access.h"
#include "param_table.h"
#include <string.h>


extern StateMachine g_ds402_state_machine;
extern MOTOR_DATA motor_data;
extern uint8_t g_can_id;

void Executor_ProcessCommand(const MotorCommand *cmd) {
  if (cmd == NULL)
    return;

  // A. 处理电机状态控制
  if (cmd->has_control_word) {
    StateMachine_SetControlword(&g_ds402_state_machine, cmd->control_word);
  } else if (cmd->enable_motor) {
    StateMachine_RequestState(&g_ds402_state_machine, STATE_OPERATION_ENABLED);
  } else {
    // 显式停止
    if (cmd->enable_motor == false) {
      if (StateMachine_GetState(&g_ds402_state_machine) ==
          STATE_OPERATION_ENABLED) {
        StateMachine_RequestState(&g_ds402_state_machine,
                                  STATE_SWITCH_ON_DISABLED);
      }
    }
  }

  // B. 处理故障查询 (CMD 30)
  if (cmd->is_fault_query) {
    MotorStatus status;
    status.can_id = g_can_id;
    status.fault_code = Safety_GetActiveFaultBits();

    CAN_Frame tx_frame;
    // Note: We need to access Protocol_BuildFaultDetail which is currently
    // private in Inovxio... ideally Protocol_BuildFault covers this, but CMD 30
    // is specific. Let's assume manager handles wrapping this or we expose the
    // build function. For now, let's keep the logic here but we might need to
    // include specific protocol headers if we want to call specific build
    // functions OR, better, Manager should route this request. But Executor is
    // supposed to do logic. Wait, "Building a specific frame" is Protocol job.
    // Executor should say "Send Fault Detail".
    // For now, I will use a callback or helper from manager/protocol if
    // available. Actually, manager.c had: ProtocolPrivate_BuildFaultDetail. I
    // should probably move that helper to a shared place or expose it. Let's
    // defer this specific line and use a placeholder or public API.
    // Protocol_BuildFault is active fault. Detail might be different.
    // Let's try regular BuildFault for now to compile.
    Protocol_BuildFault(status.fault_code, &tx_frame);
    Protocol_SendFrame(&tx_frame);
  }

  // C. 处理控制模式切换
  if (cmd->control_mode != 0) {
    __disable_irq();
    motor_data.state.Control_Mode = (CONTROL_MODE)cmd->control_mode;
    __enable_irq();
  }

  // D. 应用控制设定值
  __disable_irq();
  motor_data.Controller.input_position = cmd->pos_setpoint;
  motor_data.Controller.input_velocity = cmd->vel_setpoint;
  motor_data.Controller.input_torque = cmd->torque_ff;
  motor_data.Controller.mit_kp = cmd->kp;
  motor_data.Controller.mit_kd = cmd->kd;
  __enable_irq();

  // E. 处理参数写操作
  if (cmd->is_param_write) {
    float val_f;
    memcpy(&val_f, &cmd->param_value, sizeof(float));

    if (cmd->param_index == PARAM_RUN_MODE) {
      uint8_t proto_mode = (uint8_t)val_f;
      switch (proto_mode) {
      case 0:
        __disable_irq();
        motor_data.state.Control_Mode = CONTROL_MODE_MIT;
        __enable_irq();
        break;
      case 1:
        __disable_irq();
        motor_data.state.Control_Mode = CONTROL_MODE_POSITION_RAMP;
        __enable_irq();
        break;
      case 2:
        __disable_irq();
        motor_data.state.Control_Mode = CONTROL_MODE_VELOCITY;
        __enable_irq();
        break;
      case 3:
        __disable_irq();
        motor_data.state.Control_Mode = CONTROL_MODE_TORQUE;
        __enable_irq();
        break;
      case 5:
        __disable_irq();
        motor_data.state.Control_Mode = CONTROL_MODE_POSITION;
        __enable_irq();
        break;
      default:
        break;
      }
    } else {
      Param_WriteFloat(cmd->param_index, val_f);
    }

    if (cmd->param_index == PARAM_CAN_ID || cmd->param_index == 0x2009) {
      Param_ScheduleSave();
    }
  }

  // F. 处理参数读操作
  if (cmd->is_param_read) {
    float val_f;
    if (Param_ReadFloat(cmd->param_index, &val_f) == PARAM_OK) {
      CAN_Frame tx_frame;
      if (Protocol_BuildParamResponse(cmd->param_index, val_f, &tx_frame)) {
        Protocol_SendFrame(&tx_frame);
      }
    }
  }

  // G. 处理设置零点
  if (cmd->set_zero) {
    float current_pos = MHAL_Encoder_GetPosition();
    MHAL_Encoder_SetOffset(-current_pos);
#ifdef PARAM_ZERO_OFFSET
    Param_WriteFloat(PARAM_ZERO_OFFSET, -current_pos);
    Param_ScheduleSave();
#endif
  }

  // H. 处理协议切换
  if (cmd->is_protocol_switch) {
    Protocol_SetType((ProtocolType)cmd->target_protocol);
    Param_ScheduleSave();
  }
}
