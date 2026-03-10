/**
 * @file executor.c
 * @brief Command Executor Implementation
 */
#include "executor.h"
#include "error_manager.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"
#include "inovxio_protocol.h"
#include "manager.h"
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

static bool Executor_MapRunMode(uint8_t proto_mode, CONTROL_MODE *new_mode) {
  if (new_mode == NULL) {
    return false;
  }

  switch (proto_mode) {
  case 0:
    *new_mode = CONTROL_MODE_MIT;
    return true;
  case 1:
    *new_mode = CONTROL_MODE_POSITION_RAMP;
    return true;
  case 2:
    *new_mode = CONTROL_MODE_VELOCITY;
    return true;
  case 3:
    *new_mode = CONTROL_MODE_TORQUE;
    return true;
  case 5:
    *new_mode = CONTROL_MODE_POSITION;
    return true;
  default:
    return false;
  }
}

static ParamResult Executor_WriteParamValue(uint16_t index, float value) {
  const ParamEntry *entry = NULL;

  if (Param_GetInfo(index, &entry) != PARAM_OK || entry == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    return Param_WriteFloat(index, value);
  case PARAM_TYPE_UINT8: {
    uint8_t tmp = (uint8_t)value;
    return Param_Write(index, &tmp);
  }
  case PARAM_TYPE_UINT16: {
    uint16_t tmp = (uint16_t)value;
    return Param_Write(index, &tmp);
  }
  case PARAM_TYPE_UINT32: {
    uint32_t tmp = (uint32_t)value;
    return Param_Write(index, &tmp);
  }
  case PARAM_TYPE_INT32: {
    int32_t tmp = (int32_t)value;
    return Param_Write(index, &tmp);
  }
  default:
    return PARAM_ERR_INVALID_TYPE;
  }
}

static ParamResult Executor_ReadParamValue(uint16_t index, float *value) {
  const ParamEntry *entry = NULL;

  if (value == NULL) {
    return PARAM_ERR_NULL_PTR;
  }

  if (Param_GetInfo(index, &entry) != PARAM_OK || entry == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    *value = *(float *)entry->ptr;
    return PARAM_OK;
  case PARAM_TYPE_UINT8:
    *value = (float)*(uint8_t *)entry->ptr;
    return PARAM_OK;
  case PARAM_TYPE_UINT16:
    *value = (float)*(uint16_t *)entry->ptr;
    return PARAM_OK;
  case PARAM_TYPE_UINT32:
    *value = (float)*(uint32_t *)entry->ptr;
    return PARAM_OK;
  case PARAM_TYPE_INT32:
    *value = (float)*(int32_t *)entry->ptr;
    return PARAM_OK;
  default:
    return PARAM_ERR_INVALID_TYPE;
  }
}

static void Executor_HandleParamWrite(const MotorCommand *cmd) {
  float value = 0.0f;

  memcpy(&value, &cmd->param_value, sizeof(value));

  if (cmd->param_index == PARAM_RUN_MODE) {
    CONTROL_MODE new_mode = CONTROL_MODE_OPEN;
    uint8_t proto_mode = (uint8_t)value;

    Param_WriteUint8(PARAM_RUN_MODE, proto_mode);
    if (Executor_MapRunMode(proto_mode, &new_mode)) {
      __disable_irq();
      motor_data.state.Control_Mode = new_mode;
      __enable_irq();
    }
    return;
  }

  if (Executor_WriteParamValue(cmd->param_index, value) != PARAM_OK) {
    return;
  }

  if (cmd->param_index == PARAM_PROTOCOL_TYPE &&
      g_protocol_type <= PROTOCOL_MIT) {
    Protocol_SetType((ProtocolType)g_protocol_type);
  }

  if (cmd->param_index == PARAM_CAN_ID ||
      cmd->param_index == PARAM_CAN_BAUDRATE ||
      cmd->param_index == PARAM_PROTOCOL_TYPE) {
    Param_ScheduleSave();
  }
}

static void Executor_HandleParamRead(const MotorCommand *cmd) {
  float value = 0.0f;

  if (Executor_ReadParamValue(cmd->param_index, &value) != PARAM_OK) {
    return;
  }

  CAN_Frame tx_frame = {0};
  if (Protocol_BuildParamResponse(cmd->param_index, value, &tx_frame)) {
    Protocol_SendFrame(&tx_frame);
  }
}

void Executor_ProcessCommand(const MotorCommand *cmd) {
  if (cmd == NULL) {
    return;
  }

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

  if (cmd->is_fault_query) {
    MotorStatus status = {0};
    CAN_Frame tx_frame = {0};
    bool frame_ready = false;

    status.can_id = g_can_id;
    status.fault_code = Safety_GetActiveFaultBits();

    if (Protocol_GetType() == PROTOCOL_INOVXIO) {
      frame_ready = ProtocolPrivate_BuildFaultDetail(&status, &tx_frame);
    } else {
      frame_ready = Protocol_BuildFault(status.fault_code, &tx_frame);
    }

    if (frame_ready) {
      Protocol_SendFrame(&tx_frame);
    }
  }

  __disable_irq();
  Executor_ApplyControlCommand(cmd);
  __enable_irq();

  if (cmd->is_param_write) {
    Executor_HandleParamWrite(cmd);
  }

  if (cmd->is_param_read) {
    Executor_HandleParamRead(cmd);
  }

  if (cmd->set_zero) {
    float current_pos = MHAL_Encoder_GetPosition();
    MHAL_Encoder_SetOffset(-current_pos);
#ifdef PARAM_ZERO_OFFSET
    Param_WriteFloat(PARAM_ZERO_OFFSET, -current_pos);
    Param_ScheduleSave();
#endif
  }

  if (cmd->is_protocol_switch && cmd->target_protocol <= PROTOCOL_MIT) {
    g_protocol_type = cmd->target_protocol;
    Protocol_SetType((ProtocolType)g_protocol_type);
    Param_WriteUint8(PARAM_PROTOCOL_TYPE, g_protocol_type);
    Param_ScheduleSave();
  }
}
