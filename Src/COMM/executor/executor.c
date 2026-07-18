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
#include "fault_detection.h"
#include "fsm.h"
#include "hal_encoder.h" // For MHAL_Encoder_* ( HAL)
#include "vector_protocol.h"
#include "manager.h" // For Protocol_SendFrame used in feedback
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "platform.h"
#include "rtos/cmd_service.h"
#include "safety_control.h"
#include <stdint.h>
#include <string.h>
extern StateMachine g_ds402_state_machine;
extern MOTOR_DATA motor_data;
extern uint8_t g_can_id;

static void Executor_ApplyControlCommand(const MotorCommand *cmd) {
  CONTROL_MODE requested_mode = (CONTROL_MODE)cmd->control_mode;
  bool mode_requested = cmd->has_control_mode || cmd->control_mode != 0U;
  /* Existing CAN protocol parsers encode the mode and its setpoint in one
   * command, while USB can now change a mode independently.  Preserve the
   * old combined-command behavior without treating an omitted USB setpoint
   * as an explicit zero. */
  bool legacy_mode_command = !cmd->has_control_mode &&
                             cmd->control_mode != CONTROL_MODE_OPEN;
  bool position_requested =
      cmd->has_position_ref ||
      (legacy_mode_command &&
       (requested_mode == CONTROL_MODE_POSITION ||
        requested_mode == CONTROL_MODE_POSITION_RAMP));
  bool velocity_requested =
      cmd->has_velocity_ref ||
      (legacy_mode_command &&
       (requested_mode == CONTROL_MODE_VELOCITY ||
        requested_mode == CONTROL_MODE_VELOCITY_RAMP));
  bool torque_requested =
      cmd->has_torque_ref ||
      (legacy_mode_command && requested_mode == CONTROL_MODE_TORQUE);

  if (mode_requested) {
    motor_data.state.Control_Mode = requested_mode;
  }

  if (cmd->has_iq_ref) {
    motor_data.algo_input.Iq_ref = cmd->iq_ref;
  }
  if (cmd->has_id_ref) {
    motor_data.algo_input.Id_ref = cmd->id_ref;
  }
  if (cmd->has_torque_ref) {
    motor_data.Controller.input_torque = cmd->iq_ref;
  }
  if (position_requested) {
    /* Protocol boundary uses SI radians; the control core stores turns. */
    motor_data.Controller.input_position =
        Protocol_RadiansToTurns(cmd->position_ref);
    motor_data.Controller.input_updated = true;
  }
  if (velocity_requested) {
    /* Protocol boundary uses rad/s; the control core stores turn/s. */
    motor_data.Controller.input_velocity =
        Protocol_RadiansToTurns(cmd->speed_ref);
  }

  switch (requested_mode) {
  case CONTROL_MODE_MIT:
    motor_data.state.Control_Mode = CONTROL_MODE_MIT;
    motor_data.Controller.mit_pos_des = cmd->pos_setpoint;
    motor_data.Controller.mit_vel_des = cmd->vel_setpoint;
    motor_data.Controller.input_torque = cmd->torque_ff;
    motor_data.Controller.mit_kp = cmd->kp;
    motor_data.Controller.mit_kd = cmd->kd;
    break;
  case CONTROL_MODE_POSITION:
  case CONTROL_MODE_POSITION_RAMP:
    break;
  case CONTROL_MODE_VELOCITY:
  case CONTROL_MODE_VELOCITY_RAMP:
    break;
  case CONTROL_MODE_TORQUE:
    if (torque_requested) {
      motor_data.Controller.input_torque = cmd->iq_ref;
    }
    break;
  default:
    break;
  }
}
void Executor_ProcessCommand(const MotorCommand *cmd) {
  if (cmd == NULL)
    return;
  if (cmd->clear_fault ||
      (cmd->has_control_word && (cmd->control_word & 0x0080u) != 0u)) {
    (void)Motor_ClearFaults(&motor_data);
  }
  /* Publish setpoint/mode and the associated state request atomically so the
   * fast ISR can never observe a newly-enabled drive with the previous
   * command. */
  CRITICAL_SECTION_BEGIN();
  Executor_ApplyControlCommand(cmd);
  if (cmd->has_control_word) {
    StateMachine_SetControlword(&g_ds402_state_machine, cmd->control_word);
  } else if (cmd->has_enable_command) {
    if (cmd->enable_motor) {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_OPERATION_ENABLED);
    } else if (StateMachine_GetState(&g_ds402_state_machine) ==
               STATE_OPERATION_ENABLED) {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
  }
  CRITICAL_SECTION_END();
  // B. fault (CMD 30)
  if (cmd->is_fault_query) {
    MotorStatus status;
    status.can_id = g_can_id;
    status.fault_code = Safety_GetActiveFaultBits();
    CAN_Frame tx_frame = {0};
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
  // E. parameter write
  if (cmd->is_param_write) {
    float value = 0.0f;
    memcpy(&value, &cmd->param_value, sizeof(value));
    const ParamEntry *entry = NULL;
    bool need_save = Param_GetInfo(cmd->param_index, &entry) == PARAM_OK &&
                     entry != NULL && entry->need_save;
    if (need_save && !CmdService_BeginScheduledSave()) {
      ErrorManager_Report(ERROR_PARAM_FLASH_WRITE, "Param write save busy");
      return;
    }
    ParamResult write_result = Param_WriteFromFloat(cmd->param_index, value);
    if (need_save) {
      if (write_result == PARAM_OK) {
        CmdService_CommitScheduledSave();
      } else {
        CmdService_CancelScheduledSave();
      }
    }
  }
  // F. parameter read
  if (cmd->is_param_read) {
    float value = 0.0f;
    if (Param_ReadAsFloat(cmd->param_index, &value) == PARAM_OK) {
      CAN_Frame tx_frame = {0};
      if (Protocol_BuildParamResponse(cmd->param_index, value, &tx_frame)) {
        Protocol_SendFrame(&tx_frame);
      }
    }
  }
  // G. set
  if (cmd->set_zero) {
    CRITICAL_SECTION_BEGIN();
    if (MHAL_Encoder_ZeroPosition() == 0) {
      motor_data.feedback.position = 0.0f;
      motor_data.Controller.input_position = 0.0f;
      motor_data.Controller.pos_setpoint = 0.0f;
      motor_data.Controller.mit_pos_des = 0.0f;
      motor_data.Controller.input_updated = true;
    }
    CRITICAL_SECTION_END();
  }
  // H.
  if (cmd->is_protocol_switch) {
    if (cmd->target_protocol <= PROTOCOL_MIT) {
      if (!CmdService_BeginScheduledSave()) {
        ErrorManager_Report(ERROR_PARAM_FLASH_WRITE,
                            "Protocol switch save busy");
        return;
      }
      if (Param_WriteUint8(PARAM_PROTOCOL_TYPE, cmd->target_protocol) ==
          PARAM_OK) {
        CmdService_CommitScheduledSave();
      } else {
        CmdService_CancelScheduledSave();
      }
    }
  }
}
