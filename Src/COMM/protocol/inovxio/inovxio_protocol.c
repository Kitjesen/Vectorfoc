#include "inovxio_protocol.h"
#include "bootloader.h"
#include "fault_def.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "protocol_types.h"
#include "rtos/cmd_service.h"
#include "safety_control.h"
#include "stm32g4xx_hal.h"
#include "version.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GET_CMD_TYPE(id) (((id) >> 24) & 0x1F)
#define GET_ID_DATA(id) (((id) >> 8) & 0xFFFF)

static float Uint16ToFloat(uint16_t x, float min_val, float max_val) {
  float span = max_val - min_val;
  return ((float)x / 65535.0f) * span + min_val;
}

static uint16_t FloatToUint16(float x, float min_val, float max_val) {
  float span = max_val - min_val;

  if (x < min_val) {
    x = min_val;
  }
  if (x > max_val) {
    x = max_val;
  }

  return (uint16_t)(((x - min_val) / span) * 65535.0f);
}

static uint16_t BufToUint16(const uint8_t *val_ptr) {
  return (uint16_t)((val_ptr[0] << 8) | val_ptr[1]);
}

static void Uint16ToBuf(uint16_t val, uint8_t *val_ptr) {
  val_ptr[0] = (uint8_t)((val >> 8) & 0xFF);
  val_ptr[1] = (uint8_t)(val & 0xFF);
}

static ParseResult ParseMotorCtrl(const CAN_Frame *frame, MotorCommand *cmd) {
  uint16_t torque_raw = 0;
  uint16_t pos_raw = 0;
  uint16_t vel_raw = 0;
  uint16_t kp_raw = 0;
  uint16_t kd_raw = 0;

  if (frame->dlc < 8) {
    return PARSE_ERR_INVALID_FRAME;
  }

  torque_raw = GET_ID_DATA(frame->id);
  pos_raw = BufToUint16(&frame->data[0]);
  vel_raw = BufToUint16(&frame->data[2]);
  kp_raw = BufToUint16(&frame->data[4]);
  kd_raw = BufToUint16(&frame->data[6]);

  cmd->torque_ff = Uint16ToFloat(torque_raw, -120.0f, 120.0f);
  cmd->pos_setpoint = Uint16ToFloat(pos_raw, -12.57f, 12.57f);
  cmd->vel_setpoint = Uint16ToFloat(vel_raw, -15.0f, 15.0f);
  cmd->kp = Uint16ToFloat(kp_raw, 0.0f, 500.0f);
  cmd->kd = Uint16ToFloat(kd_raw, 0.0f, 100.0f);
  cmd->control_mode = CONTROL_MODE_MIT;
  cmd->has_enable_command = true;
  cmd->enable_motor = true;
  return PARSE_OK;
}

static ParseResult ParseParamCommand(const CAN_Frame *frame, MotorCommand *cmd,
                                     bool is_write) {
  if (frame->dlc < 4) {
    return PARSE_ERR_INVALID_FRAME;
  }

  cmd->param_index = (uint16_t)(frame->data[0] | (frame->data[1] << 8));
  if (is_write) {
    if (frame->dlc < 8) {
      return PARSE_ERR_INVALID_FRAME;
    }

    memcpy(&cmd->param_value, &frame->data[4], 4);
    cmd->is_param_write = true;
  } else {
    cmd->is_param_read = true;
  }

  return PARSE_OK;
}

ParseResult ProtocolPrivate_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    return PARSE_ERR_INVALID_FRAME;
  }

  if (!frame->is_extended) {
    return PARSE_ERR_INVALID_FRAME;
  }

  memset(cmd, 0, sizeof(*cmd));

  switch ((PrivateCmdType)GET_CMD_TYPE(frame->id)) {
  case PRIVATE_CMD_MOTOR_CTRL:
    return ParseMotorCtrl(frame, cmd);

  case PRIVATE_CMD_MOTOR_ENABLE:
    cmd->has_enable_command = true;
    cmd->enable_motor = true;
    return PARSE_OK;

  case PRIVATE_CMD_MOTOR_STOP:
    cmd->has_enable_command = true;
    cmd->enable_motor = false;
    return PARSE_OK;

  case PRIVATE_CMD_SET_ZERO:
    cmd->set_zero = true;
    return PARSE_OK;

  case PRIVATE_CMD_CALIBRATE: {
    uint8_t type = 0;
    if (frame->dlc >= 1) {
      type = frame->data[0];
    }
    Motor_RequestCalibration(&motor_data, type);
    return PARSE_OK;
  }

  case PRIVATE_CMD_SET_ID:
    if (frame->dlc >= 4) {
      Param_WriteUint8(PARAM_CAN_ID, frame->data[0]);
      Param_ScheduleSave();
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;

  case PRIVATE_CMD_PARAM_READ:
    return ParseParamCommand(frame, cmd, false);

  case PRIVATE_CMD_PARAM_WRITE:
    return ParseParamCommand(frame, cmd, true);

  case PRIVATE_CMD_SAVE:
    Param_ScheduleSave();
    return PARSE_OK;

  case PRIVATE_CMD_GET_VERSION: {
    const FirmwareVersion_t *version = FW_GetVersion();
    CAN_Frame tx_frame = {0};

    tx_frame.id = ((uint32_t)PRIVATE_CMD_GET_VERSION << 24) | (g_can_id << 8) |
                  0xFD;
    tx_frame.is_extended = true;
    tx_frame.dlc = 8;
    tx_frame.data[0] = version->major;
    tx_frame.data[1] = version->minor;
    tx_frame.data[2] = version->patch;
    tx_frame.data[3] = version->dirty;
    tx_frame.data[4] = (uint8_t)version->git_hash[0];
    tx_frame.data[5] = (uint8_t)version->git_hash[1];
    tx_frame.data[6] = (uint8_t)version->git_hash[2];
    tx_frame.data[7] = (uint8_t)version->git_hash[3];
    Protocol_SendFrame(&tx_frame);
    return PARSE_OK;
  }

  case PRIVATE_CMD_FAULT_QUERY:
    cmd->is_fault_query = true;
    return PARSE_OK;

  case PRIVATE_CMD_RESET:
    HAL_NVIC_SystemReset();
    return PARSE_OK;

  case PRIVATE_CMD_CLEAR_FAULT:
    Safety_ClearFaults(&g_ds402_state_machine);
    Motor_ClearFaults(&motor_data);
    return PARSE_OK;

  case PRIVATE_CMD_BOOTLOADER:
    Boot_RequestUpgrade();
    return PARSE_OK;

  case PRIVATE_CMD_REPORT:
    if (frame->dlc >= 1) {
      CmdService_SetReportEnable(frame->data[0] != 0);
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;

  case PRIVATE_CMD_SET_BAUDRATE:
    if (frame->dlc >= 1) {
      Param_WriteUint8(PARAM_CAN_BAUDRATE, frame->data[0]);
      Param_ScheduleSave();
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;

  case PRIVATE_CMD_SET_PROTOCOL:
    if (frame->dlc >= 1) {
      cmd->is_protocol_switch = true;
      cmd->target_protocol = frame->data[0];
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;

  default:
    return PARSE_ERR_UNSUPPORTED;
  }
}

bool ProtocolPrivate_BuildFeedback(const MotorStatus *status,
                                   CAN_Frame *frame) {
  uint32_t mode = 2;
  uint32_t fault_bits = 0;
  uint32_t id = 0;
  uint16_t pos = 0;
  uint16_t vel = 0;
  uint16_t tor = 0;
  int16_t temp = 0;

  if (status == NULL || frame == NULL) {
    return false;
  }

  if (status->fault_code & FAULT_ENCODER_UNCALIBRATED) {
    fault_bits |= (1U << 5);
  }
  if (status->fault_code & FAULT_STALL_OVERLOAD) {
    fault_bits |= (1U << 4);
  }
  if (status->fault_code & FAULT_HARDWARE_ID) {
    fault_bits |= (1U << 3);
  }
  if (status->fault_code & FAULT_OVER_TEMP) {
    fault_bits |= (1U << 2);
  }
  if (status->fault_code &
      (FAULT_CURRENT_A | FAULT_CURRENT_B | FAULT_CURRENT_C)) {
    fault_bits |= (1U << 1);
  }
  if (status->fault_code & FAULT_UNDER_VOLTAGE) {
    fault_bits |= (1U << 0);
  }

  id = (0x02U << 24) | (mode << 22) | (fault_bits << 16) |
       (status->can_id << 8) | 0xFDU;
  frame->id = id;
  frame->is_extended = true;
  frame->dlc = 8;

  pos = FloatToUint16(status->position, -12.57f, 12.57f);
  vel = FloatToUint16(status->velocity, -15.0f, 15.0f);
  tor = FloatToUint16(status->torque, -120.0f, 120.0f);
  temp = (int16_t)(status->temperature * 10.0f);

  Uint16ToBuf(pos, &frame->data[0]);
  Uint16ToBuf(vel, &frame->data[2]);
  Uint16ToBuf(tor, &frame->data[4]);
  Uint16ToBuf((uint16_t)temp, &frame->data[6]);
  return true;
}

bool ProtocolPrivate_BuildFault(uint32_t fault_code, uint32_t warning_code,
                                CAN_Frame *frame) {
  (void)warning_code;

  if (frame == NULL) {
    return false;
  }

  frame->id = (0x15U << 24) | (g_can_id << 8) | 0xFDU;
  frame->is_extended = true;
  frame->dlc = 8;
  frame->data[0] = (uint8_t)((fault_code >> 24) & 0xFF);
  frame->data[1] = (uint8_t)((fault_code >> 16) & 0xFF);
  frame->data[2] = (uint8_t)((fault_code >> 8) & 0xFF);
  frame->data[3] = (uint8_t)(fault_code & 0xFF);
  return true;
}

void ProtocolPrivate_Init(void) {}

bool ProtocolPrivate_BuildParamResponse(uint16_t param_index, float value,
                                        CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  frame->id = (0x12U << 24) | (g_can_id << 8) | 0xFDU;
  frame->is_extended = true;
  frame->dlc = 8;
  frame->data[0] = (uint8_t)(param_index & 0xFF);
  frame->data[1] = (uint8_t)((param_index >> 8) & 0xFF);
  frame->data[2] = 0;
  frame->data[3] = 0;
  memcpy(&frame->data[4], &value, 4);
  return true;
}

bool ProtocolPrivate_BuildFaultDetail(const MotorStatus *status,
                                      CAN_Frame *frame) {
  uint32_t timestamp = 0;

  if (status == NULL || frame == NULL) {
    return false;
  }

  frame->id = (0x1EU << 24) | (status->can_id << 8) | 0xFDU;
  frame->is_extended = true;
  frame->dlc = 8;
  frame->data[0] = (uint8_t)((status->fault_code >> 8) & 0xFF);
  frame->data[1] = (uint8_t)(status->fault_code & 0xFF);
  frame->data[2] = (uint8_t)((status->fault_code >> 24) & 0xFF);
  frame->data[3] = (uint8_t)((status->fault_code >> 16) & 0xFF);

  timestamp = Safety_GetLastFaultTime();
  frame->data[4] = (uint8_t)((timestamp >> 24) & 0xFF);
  frame->data[5] = (uint8_t)((timestamp >> 16) & 0xFF);
  frame->data[6] = (uint8_t)((timestamp >> 8) & 0xFF);
  frame->data[7] = (uint8_t)(timestamp & 0xFF);
  return true;
}
