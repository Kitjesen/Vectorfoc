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
 * @file protocol_mit.c
 * @brief MIT Cheetah
 * @version 1.0
 * @date 2026-01-20
 */
#include "mit_protocol.h"
#include "motor.h"         /* g_can_id */
#include "protocol_utils.h" /* Proto_Uint16ToFloat / FloatToUint16 / Uint12 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint8_t s_motor_can_id = 1;

/* 量化工具函数由 protocol_utils.h 提供，本地名称别名 */
#define Uint16ToFloat  Proto_Uint16ToFloat
#define FloatToUint16  Proto_FloatToUint16
#define Uint12ToFloat  Proto_Uint12ToFloat
#define FloatToUint12  Proto_FloatToUint12
/**
 * @brief initMIT
 */
void ProtocolMIT_Init(void) { s_motor_can_id = 1; }
/**
 * @brief MIT
 * MIT Cheetah (8):
 * [Position(16bit)][Velocity(12bit)][Kp(12bit)][Kd(12bit)][Torque(12bit)]
 * : 16+12+12+12+12 = 64 bits = 8 bytes
 */
static ParseResult ParseMITControl(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame->dlc < 8) {
    return PARSE_ERR_INVALID_FRAME;
  }
  // Position: Byte 0-1 (16bit)
  uint16_t pos_raw = (frame->data[0] << 8) | frame->data[1];
  cmd->pos_setpoint = Uint16ToFloat(pos_raw, MIT_P_MIN, MIT_P_MAX);
  // Velocity: Byte 2 + 4bits of Byte 3 (12bit)
  uint16_t vel_raw = (frame->data[2] << 4) | (frame->data[3] >> 4);
  cmd->vel_setpoint = Uint12ToFloat(vel_raw, MIT_V_MIN, MIT_V_MAX);
  // Kp: 4bits of Byte 3 + Byte 4 (12bit)
  uint16_t kp_raw = ((frame->data[3] & 0x0F) << 8) | frame->data[4];
  cmd->kp = Uint12ToFloat(kp_raw, MIT_KP_MIN, MIT_KP_MAX);
  // Kd: Byte 5 + 4bits of Byte 6 (12bit)
  uint16_t kd_raw = (frame->data[5] << 4) | (frame->data[6] >> 4);
  cmd->kd = Uint12ToFloat(kd_raw, MIT_KD_MIN, MIT_KD_MAX);
  // Torque: 4bits of Byte 6 + Byte 7 (12bit)
  uint16_t torque_raw = ((frame->data[6] & 0x0F) << 8) | frame->data[7];
  cmd->torque_ff = Uint12ToFloat(torque_raw, MIT_T_MIN, MIT_T_MAX);
  cmd->control_mode = CONTROL_MODE_MIT;
  cmd->enable_motor = true;
  return PARSE_OK;
}
/**
 * @brief MIT
 */
/**
 * @brief MIT 特殊命令帧（DLC=2）解析
 *
 * MIT Cheetah 协议约定：DLC=2 时为特殊命令帧。
 *   data[0] = 0xFF (magic byte，区分普通控制帧)
 *   data[1] = MITCmdType
 */
static ParseResult ParseMITSpecialCmd(const CAN_Frame *frame, MotorCommand *cmd)
{
  if (frame->dlc < 2 || frame->data[0] != 0xFF) {
    return PARSE_ERR_INVALID_FRAME;
  }
  MITCmdType type = (MITCmdType)frame->data[1];
  switch (type) {
  case MIT_CMD_MOTOR_OFF:   /* 0x00 — disable motor */
    cmd->enable_motor = false;
    return PARSE_OK;
  case MIT_CMD_MOTOR_ON:    /* 0x01 — enable motor */
    cmd->enable_motor = true;
    return PARSE_OK;
  case MIT_CMD_SET_ZERO:    /* 0x02 — set current position as zero */
    cmd->set_zero = true;
    return PARSE_OK;
  case MIT_CMD_GET_STATE:   /* 0x04 — request status frame (handled by manager) */
    /* No specific MotorCommand fields needed; manager sends feedback on PARSE_OK */
    return PARSE_OK;
  default:
    return PARSE_ERR_UNSUPPORTED;
  }
}

ParseResult ProtocolMIT_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    return PARSE_ERR_INVALID_FRAME;
  }
  memset(cmd, 0, sizeof(MotorCommand));

  if (frame->dlc == 8) {
    /* Standard MIT impedance control frame */
    return ParseMITControl(frame, cmd);
  } else if (frame->dlc == 2) {
    /* MIT special command frame: [0xFF][CmdType] */
    return ParseMITSpecialCmd(frame, cmd);
  } else if (frame->dlc == 0 && frame->is_rtr) {
    /* RTR: remote status request — manager handles feedback reply */
    return PARSE_OK;
  }

  return PARSE_ERR_UNSUPPORTED;
}
/**
 * @brief MITfeedback
 * MITfeedback (6):
 * [Position(16bit)][Velocity(12bit)][Torque(12bit)][Mode/Error(8bit)]
 */
bool ProtocolMIT_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }
  frame->id = g_can_id;
  frame->dlc = 6;
  frame->is_extended = false;
  frame->is_rtr = false;
  // Position (16bit)
  uint16_t pos_raw = FloatToUint16(status->position, MIT_P_MIN, MIT_P_MAX);
  frame->data[0] = (pos_raw >> 8) & 0xFF;
  frame->data[1] = pos_raw & 0xFF;
  // Velocity (12bit)
  uint16_t vel_raw = FloatToUint12(status->velocity, MIT_V_MIN, MIT_V_MAX);
  frame->data[2] = (vel_raw >> 4) & 0xFF;
  // Torque (12bit)
  uint16_t torque_raw = FloatToUint12(status->torque, MIT_T_MIN, MIT_T_MAX);
  frame->data[3] = ((vel_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F);
  frame->data[4] = torque_raw & 0xFF;
  // Mode + Error
  frame->data[5] =
      ((status->motor_state & 0x0F) << 4) | (status->fault_code & 0x0F);
  return true;
}
/**
 * @brief MITfault
 */
bool ProtocolMIT_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  frame->id = g_can_id;
  frame->dlc = 4;
  frame->is_extended = false;
  frame->is_rtr = false;
  // fault
  frame->data[0] = 0xFF; // fault
  frame->data[1] = (fault_code >> 16) & 0xFF;
  frame->data[2] = (fault_code >> 8) & 0xFF;
  frame->data[3] = fault_code & 0xFF;
  return true;
}
