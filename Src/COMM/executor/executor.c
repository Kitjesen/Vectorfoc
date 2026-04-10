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
 * @file executor.c
 * @brief Command Executor Implementation
 */
#include "executor.h"
#include "error_manager.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h" // For MHAL_Encoder_* ( HAL)
#include "vector_protocol.h"
#include "manager.h" // For Protocol_SendFrame used in feedback
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "safety_control.h"
#include <string.h>
extern StateMachine g_ds402_state_machine;
extern MOTOR_DATA motor_data;
extern uint8_t g_can_id;

static void Executor_ApplyControlCommand(const MotorCommand *cmd) {
  if (cmd->control_mode == 0) {
    return;
  }

  motor_data.state.Control_Mode = (CONTROL_MODE)cmd->control_mode;

  switch ((CONTROL_MODE)cmd->control_mode) {
  case CONTROL_MODE_MIT:
    motor_data.Controller.mit_pos_des = cmd->pos_setpoint;
    motor_data.Controller.mit_vel_des = cmd->vel_setpoint;
    motor_data.Controller.input_torque = cmd->torque_ff;
    motor_data.Controller.mit_kp = cmd->kp;
    motor_data.Controller.mit_kd = cmd->kd;
    break;
  case CONTROL_MODE_POSITION:
  case CONTROL_MODE_POSITION_RAMP:
    motor_data.Controller.input_position = cmd->position_ref;
    motor_data.Controller.input_updated = true;
    break;
  case CONTROL_MODE_VELOCITY:
  case CONTROL_MODE_VELOCITY_RAMP:
    motor_data.Controller.input_velocity = cmd->speed_ref;
    break;
  case CONTROL_MODE_TORQUE:
    motor_data.Controller.input_torque = cmd->iq_ref;
    break;
  default:
    break;
  }
}
void Executor_ProcessCommand(const MotorCommand *cmd) {
  if (cmd == NULL)
    return;
  // A. motorstate
  if (cmd->has_control_word) {
    StateMachine_SetControlword(&g_ds402_state_machine, cmd->control_word);
  } else if (cmd->enable_motor) {
    StateMachine_RequestState(&g_ds402_state_machine, STATE_OPERATION_ENABLED);
  } else {
    // stop
    if (cmd->enable_motor == false) {
      if (StateMachine_GetState(&g_ds402_state_machine) ==
          STATE_OPERATION_ENABLED) {
        StateMachine_RequestState(&g_ds402_state_machine,
                                  STATE_SWITCH_ON_DISABLED);
      }
    }
  }
  // B. fault (CMD 30)
  if (cmd->is_fault_query) {
    MotorStatus status;
    status.can_id = g_can_id;
    status.fault_code = Safety_GetActiveFaultBits();
    CAN_Frame tx_frame;
    bool frame_ready = false;
    if (Protocol_GetType() == PROTOCOL_VECTOR) {
      frame_ready = ProtocolVector_BuildFaultDetail(&status, &tx_frame);
    } else {
      frame_ready = Protocol_BuildFault(status.fault_code, &tx_frame);
    }
    if (frame_ready) {
      Protocol_SendFrame(&tx_frame);
    }
  }
  /* [FIX] C+D ，mode
   *  __disable_irq ， ISR
   * modeupdatestate */
  __disable_irq();
  Executor_ApplyControlCommand(cmd);
  __enable_irq();
  // E. param
  if (cmd->is_param_write) {
    float val_f;
    memcpy(&val_f, &cmd->param_value, sizeof(float));
    if (cmd->param_index == PARAM_RUN_MODE) {
      /* [FIX] mode
       *  case  disable/enable_irq， */
      CONTROL_MODE new_mode = (CONTROL_MODE)(-1);
      uint8_t proto_mode = (uint8_t)val_f;
      switch (proto_mode) {
      case 0:
        new_mode = CONTROL_MODE_MIT;
        break;
      case 1:
        new_mode = CONTROL_MODE_POSITION_RAMP;
        break;
      case 2:
        new_mode = CONTROL_MODE_VELOCITY;
        break;
      case 3:
        new_mode = CONTROL_MODE_TORQUE;
        break;
      case 5:
        new_mode = CONTROL_MODE_POSITION;
        break;
      default:
        break;
      }
      if ((int)new_mode >= 0) {
        __disable_irq();
        motor_data.state.Control_Mode = new_mode;
        __enable_irq();
      }
    } else {
      Param_WriteFloat(cmd->param_index, val_f);
    }
    if (cmd->param_index == PARAM_CAN_ID || cmd->param_index == 0x2009) {
      Param_ScheduleSave();
    }
  }
  // F. param
  if (cmd->is_param_read) {
    float val_f;
    if (Param_ReadFloat(cmd->param_index, &val_f) == PARAM_OK) {
      CAN_Frame tx_frame;
      if (Protocol_BuildParamResponse(cmd->param_index, val_f, &tx_frame)) {
        Protocol_SendFrame(&tx_frame);
      }
    }
  }
  // G. set
  if (cmd->set_zero) {
    float current_pos = MHAL_Encoder_GetPosition();
    MHAL_Encoder_SetOffset(-current_pos);
#ifdef PARAM_ZERO_OFFSET
    Param_WriteFloat(PARAM_ZERO_OFFSET, -current_pos);
    Param_ScheduleSave();
#endif
  }
  // H.
  if (cmd->is_protocol_switch) {
    if (cmd->target_protocol <= PROTOCOL_MIT) {
      g_protocol_type = cmd->target_protocol;
      Protocol_SetType((ProtocolType)g_protocol_type);
      Param_WriteUint8(PARAM_PROTOCOL_TYPE, g_protocol_type);
      Param_ScheduleSave();
    }
  }
}
