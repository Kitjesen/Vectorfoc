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
 * @file    canopen_protocol.c
 * @brief   CANopen DS402 protocol implementation.
 * @note    Provides RPDO1 parsing, expedited SDO downloads, EMCY, and
 *          heartbeat.
 */
#include "canopen_protocol.h"
#include "manager.h"
#include "motor.h"
#include <limits.h>
#include <math.h>
#include <string.h>

/* CANopen default communication object base identifiers. */
#define CANOPEN_FC_NMT 0x000U
#define CANOPEN_FC_EMCY 0x080U
#define CANOPEN_FC_TPDO1 0x180U
#define CANOPEN_FC_RPDO1 0x200U
#define CANOPEN_FC_SDO_RX 0x600U
#define CANOPEN_FC_SDO_TX 0x580U
#define CANOPEN_FC_HEARTBEAT 0x700U

/* NMT commands. */
#define CANOPEN_NMT_START 0x01U
#define CANOPEN_NMT_STOP 0x02U
#define CANOPEN_NMT_ENTER_PREOP 0x80U
#define CANOPEN_NMT_RESET_NODE 0x81U
#define CANOPEN_NMT_RESET_COMM 0x82U

/* Expedited SDO download command bytes (CiA 301). */
#define CANOPEN_SDO_DOWNLOAD_1BYTE 0x2FU
#define CANOPEN_SDO_DOWNLOAD_2BYTE 0x2BU
#define CANOPEN_SDO_DOWNLOAD_3BYTE 0x27U
#define CANOPEN_SDO_DOWNLOAD_4BYTE 0x23U
#define CANOPEN_SDO_DOWNLOAD_RESPONSE 0x60U
#define CANOPEN_SDO_ABORT_RESPONSE 0x80U

/* SDO abort codes used by this compact object dictionary. */
#define CANOPEN_SDO_ABORT_COMMAND 0x05040001UL
#define CANOPEN_SDO_ABORT_OBJECT 0x06020000UL
#define CANOPEN_SDO_ABORT_LENGTH 0x06070010UL
#define CANOPEN_SDO_ABORT_SUBINDEX 0x06090011UL
#define CANOPEN_SDO_ABORT_VALUE 0x06090030UL

static uint8_t s_node_id = 1U;
static CANopenNodeState s_node_state = CANOPEN_STATE_PREOPERATIONAL;
static CANopenMode s_current_mode = CANOPEN_MODE_CST;
static uint32_t s_last_hb_ms = 0U;

static uint16_t BufToU16(const uint8_t *buf) {
  return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static int16_t BufToI16(const uint8_t *buf) {
  return (int16_t)BufToU16(buf);
}

static int32_t BufToI32(const uint8_t *buf) {
  uint32_t raw = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                 ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
  return (int32_t)raw;
}

static void U32ToBuf(uint32_t value, uint8_t *buf) {
  buf[0] = (uint8_t)value;
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)(value >> 16);
  buf[3] = (uint8_t)(value >> 24);
}

static float PositionRawToSI(int32_t raw) {
  return (float)raw / CANOPEN_POSITION_UNITS_PER_RADIAN;
}

static float VelocityRawToSI(int32_t raw) {
  return (float)raw / CANOPEN_VELOCITY_UNITS_PER_RADIAN;
}

static int32_t MotionSIToRaw(float value, float units_per_radian) {
  if (!isfinite(value) || !isfinite(units_per_radian) ||
      units_per_radian <= 0.0f) {
    return 0;
  }
  float scaled = value * units_per_radian;
  if (scaled > (float)INT32_MAX) {
    return INT32_MAX;
  }
  if (scaled < (float)INT32_MIN) {
    return INT32_MIN;
  }
  return (int32_t)scaled;
}

static bool CANopen_IsSupportedMode(CANopenMode mode) {
  switch (mode) {
  case CANOPEN_MODE_POSITION:
  case CANOPEN_MODE_VELOCITY:
  case CANOPEN_MODE_TORQUE:
  case CANOPEN_MODE_CSP:
  case CANOPEN_MODE_CSV:
  case CANOPEN_MODE_CST:
    return true;
  default:
    return false;
  }
}

static float CANopen_TorquePerMilleToNm(int16_t target) {
  float torque_limit = motor_data.Controller.torque_limit;
  if (!isfinite(torque_limit) || torque_limit <= 0.0f) {
    return 0.0f;
  }
  return (float)target * torque_limit / 1000.0f;
}

static void CANopen_SendSDOResponse(uint16_t index, uint8_t subindex,
                                    uint32_t abort_code) {
  CAN_Frame response = {0};
  response.id = CANOPEN_FC_SDO_TX + s_node_id;
  response.dlc = 8U;
  response.data[0] = abort_code == 0U ? CANOPEN_SDO_DOWNLOAD_RESPONSE
                                     : CANOPEN_SDO_ABORT_RESPONSE;
  response.data[1] = (uint8_t)index;
  response.data[2] = (uint8_t)(index >> 8);
  response.data[3] = subindex;
  if (abort_code != 0U) {
    U32ToBuf(abort_code, &response.data[4]);
  }
  (void)Protocol_SendFrame(&response);
}

static ParseResult CANopen_ParseSDODownload(const CAN_Frame *frame,
                                            MotorCommand *cmd) {
  uint16_t index = frame->dlc >= 3U ? BufToU16(&frame->data[1]) : 0U;
  uint8_t subindex = frame->dlc >= 4U ? frame->data[3] : 0U;
  if (frame->dlc != 8U) {
    if (frame->dlc >= 4U) {
      CANopen_SendSDOResponse(index, subindex, CANOPEN_SDO_ABORT_LENGTH);
    }
    return PARSE_ERR_INVALID_FRAME;
  }
  if (subindex != 0U) {
    CANopen_SendSDOResponse(index, subindex, CANOPEN_SDO_ABORT_SUBINDEX);
    return PARSE_ERR_UNSUPPORTED;
  }

  uint8_t expected_command;
  switch (index) {
  case CANOPEN_OBJ_CONTROLWORD:
  case CANOPEN_OBJ_TARGET_TORQUE:
    expected_command = CANOPEN_SDO_DOWNLOAD_2BYTE;
    break;
  case CANOPEN_OBJ_MODES_OF_OP:
    expected_command = CANOPEN_SDO_DOWNLOAD_1BYTE;
    break;
  case CANOPEN_OBJ_TARGET_POSITION:
  case CANOPEN_OBJ_TARGET_VELOCITY:
    expected_command = CANOPEN_SDO_DOWNLOAD_4BYTE;
    break;
  default:
    CANopen_SendSDOResponse(index, subindex, CANOPEN_SDO_ABORT_OBJECT);
    return PARSE_ERR_UNSUPPORTED;
  }

  uint8_t command = frame->data[0];
  if (command != CANOPEN_SDO_DOWNLOAD_1BYTE &&
      command != CANOPEN_SDO_DOWNLOAD_2BYTE &&
      command != CANOPEN_SDO_DOWNLOAD_3BYTE &&
      command != CANOPEN_SDO_DOWNLOAD_4BYTE) {
    CANopen_SendSDOResponse(index, subindex, CANOPEN_SDO_ABORT_COMMAND);
    return PARSE_ERR_UNSUPPORTED;
  }
  if (command != expected_command) {
    CANopen_SendSDOResponse(index, subindex, CANOPEN_SDO_ABORT_LENGTH);
    return PARSE_ERR_INVALID_FRAME;
  }

  switch (index) {
  case CANOPEN_OBJ_CONTROLWORD:
    cmd->has_control_word = true;
    cmd->control_word = BufToU16(&frame->data[4]);
    break;
  case CANOPEN_OBJ_MODES_OF_OP: {
    CANopenMode mode = (CANopenMode)(int8_t)frame->data[4];
    if (!CANopen_IsSupportedMode(mode)) {
      CANopen_SendSDOResponse(index, subindex, CANOPEN_SDO_ABORT_VALUE);
      return PARSE_ERR_UNSUPPORTED;
    }
    s_current_mode = mode;
    break;
  }
  case CANOPEN_OBJ_TARGET_POSITION:
    cmd->control_mode = CONTROL_MODE_POSITION;
    cmd->position_ref = PositionRawToSI(BufToI32(&frame->data[4]));
    break;
  case CANOPEN_OBJ_TARGET_VELOCITY:
    cmd->control_mode = CONTROL_MODE_VELOCITY;
    cmd->speed_ref = VelocityRawToSI(BufToI32(&frame->data[4]));
    break;
  case CANOPEN_OBJ_TARGET_TORQUE:
    cmd->control_mode = CONTROL_MODE_TORQUE;
    cmd->iq_ref = CANopen_TorquePerMilleToNm(BufToI16(&frame->data[4]));
    break;
  default:
    return PARSE_ERR_UNSUPPORTED;
  }

  CANopen_SendSDOResponse(index, subindex, 0U);
  return PARSE_OK;
}

void ProtocolCANopen_Init(void) {
  s_node_id = (g_can_id >= 1U && g_can_id <= 127U) ? g_can_id : 1U;
  s_current_mode = CANOPEN_MODE_CST;
  s_node_state = CANOPEN_STATE_PREOPERATIONAL;
  s_last_hb_ms = 0U;
}

ParseResult ProtocolCANopen_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    return PARSE_ERR_INVALID_FRAME;
  }
  memset(cmd, 0, sizeof(MotorCommand));
  if (frame->is_extended || frame->is_rtr) {
    return PARSE_UNKNOWN_ID;
  }
  if (frame->id == CANOPEN_FC_NMT) {
    return ProtocolCANopen_HandleNMT(frame) ? PARSE_OK : PARSE_UNKNOWN_ID;
  }

  /*
   * RPDO1 compact mapping:
   *   byte 0..1 controlword, byte 2 mode, byte 3 reserved,
   *   byte 4..7 position/velocity target or byte 4..5 target torque.
   */
  if (frame->id == (CANOPEN_FC_RPDO1 + s_node_id)) {
    if (frame->dlc != 8U) {
      return PARSE_ERR_INVALID_FRAME;
    }
    cmd->has_control_word = true;
    cmd->control_word = BufToU16(&frame->data[0]);
    s_current_mode = (CANopenMode)(int8_t)frame->data[2];
    switch (s_current_mode) {
    case CANOPEN_MODE_POSITION:
    case CANOPEN_MODE_CSP:
      cmd->control_mode = CONTROL_MODE_POSITION;
      cmd->position_ref = PositionRawToSI(BufToI32(&frame->data[4]));
      break;
    case CANOPEN_MODE_VELOCITY:
    case CANOPEN_MODE_CSV:
      cmd->control_mode = CONTROL_MODE_VELOCITY;
      cmd->speed_ref = VelocityRawToSI(BufToI32(&frame->data[4]));
      break;
    case CANOPEN_MODE_TORQUE:
    case CANOPEN_MODE_CST:
      cmd->control_mode = CONTROL_MODE_TORQUE;
      cmd->iq_ref = CANopen_TorquePerMilleToNm(BufToI16(&frame->data[4]));
      break;
    default:
      return PARSE_ERR_UNSUPPORTED;
    }
    return PARSE_OK;
  }

  if (frame->id == (CANOPEN_FC_SDO_RX + s_node_id)) {
    return CANopen_ParseSDODownload(frame, cmd);
  }
  return PARSE_UNKNOWN_ID;
}

bool ProtocolCANopen_BuildFeedback(const MotorStatus *status,
                                   CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->id = CANOPEN_FC_TPDO1 + s_node_id;
  frame->dlc = 8U;
  int32_t pos =
      MotionSIToRaw(status->position, CANOPEN_POSITION_UNITS_PER_RADIAN);
  int32_t vel =
      MotionSIToRaw(status->velocity, CANOPEN_VELOCITY_UNITS_PER_RADIAN);
  U32ToBuf((uint32_t)pos, &frame->data[0]);
  U32ToBuf((uint32_t)vel, &frame->data[4]);
  return true;
}

bool ProtocolCANopen_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  frame->id = CANOPEN_FC_EMCY + s_node_id;
  frame->dlc = 8U;
  U32ToBuf(fault_code, &frame->data[0]);
  return true;
}

bool ProtocolCANopen_HandleNMT(const CAN_Frame *frame) {
  if (frame == NULL || frame->dlc < 2U) {
    return false;
  }
  uint8_t command = frame->data[0];
  uint8_t target = frame->data[1];
  if (target != 0U && target != s_node_id) {
    return false;
  }
  switch (command) {
  case CANOPEN_NMT_START:
    s_node_state = CANOPEN_STATE_OPERATIONAL;
    break;
  case CANOPEN_NMT_STOP:
    s_node_state = CANOPEN_STATE_STOPPED;
    break;
  case CANOPEN_NMT_ENTER_PREOP:
    s_node_state = CANOPEN_STATE_PREOPERATIONAL;
    break;
  case CANOPEN_NMT_RESET_NODE:
  case CANOPEN_NMT_RESET_COMM:
    ProtocolCANopen_Init();
    break;
  default:
    return false;
  }
  return true;
}

bool ProtocolCANopen_BuildHeartbeat(uint32_t now_ms, CAN_Frame *frame) {
  if (frame == NULL || now_ms - s_last_hb_ms < 1000U) {
    return false;
  }
  memset(frame, 0, sizeof(*frame));
  s_last_hb_ms = now_ms;
  frame->id = CANOPEN_FC_HEARTBEAT + s_node_id;
  frame->dlc = 1U;
  frame->data[0] = (uint8_t)s_node_state;
  return true;
}
