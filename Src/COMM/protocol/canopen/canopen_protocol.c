/**
 * @file    canopen_protocol.c
 * @brief   CANopen DS402 protocol implementation (minimal)
 * @note    Provides basic RPDO1 parsing, EMCY, and heartbeat.
 */

#include "canopen_protocol.h"
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include <string.h>

/* CANopen 功能码 (COB-ID 基址) */
#define CANOPEN_FC_NMT 0x000
#define CANOPEN_FC_EMCY 0x080
#define CANOPEN_FC_TPDO1 0x180
#define CANOPEN_FC_RPDO1 0x200
#define CANOPEN_FC_SDO_RX 0x600
#define CANOPEN_FC_SDO_TX 0x580
#define CANOPEN_FC_HEARTBEAT 0x700

/* NMT 命令 */
#define CANOPEN_NMT_START 0x01
#define CANOPEN_NMT_STOP 0x02
#define CANOPEN_NMT_ENTER_PREOP 0x80
#define CANOPEN_NMT_RESET_NODE 0x81
#define CANOPEN_NMT_RESET_COMM 0x82

static uint8_t s_node_id = 1;
static CANopenNodeState s_node_state = CANOPEN_STATE_PREOPERATIONAL;
static CANopenMode s_current_mode = CANOPEN_MODE_CST;
static uint32_t s_last_hb_ms = 0;

static uint16_t BufToU16(const uint8_t *buf) {
  return (uint16_t)(buf[0] | (buf[1] << 8));
}

static int32_t BufToI32(const uint8_t *buf) {
  return (int32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
}

void ProtocolCANopen_Init(void) {
  s_node_id = 1;
  s_current_mode = CANOPEN_MODE_CST;
  s_node_state = CANOPEN_STATE_PREOPERATIONAL;
  s_last_hb_ms = 0;
}

ParseResult ProtocolCANopen_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    return PARSE_ERR_INVALID_FRAME;
  }

  memset(cmd, 0, sizeof(MotorCommand));

  if (frame->id == CANOPEN_FC_NMT) {
    return ProtocolCANopen_HandleNMT(frame) ? PARSE_OK : PARSE_UNKNOWN_ID;
  }

  /* RPDO1: 控制字 + 模式 + 目标值 */
  if (frame->id == (CANOPEN_FC_RPDO1 + s_node_id)) {
    if (frame->dlc < 8) {
      return PARSE_ERR_INVALID_FRAME;
    }

    cmd->has_control_word = true;
    cmd->control_word = BufToU16(&frame->data[0]);
    s_current_mode = (CANopenMode)frame->data[2];

    int32_t target = BufToI32(&frame->data[4]);
    switch (s_current_mode) {
    case CANOPEN_MODE_POSITION:
      cmd->control_mode = CONTROL_MODE_POSITION;
      cmd->position_ref = (float)target;
      break;
    case CANOPEN_MODE_VELOCITY:
      cmd->control_mode = CONTROL_MODE_VELOCITY;
      cmd->speed_ref = (float)target;
      break;
    case CANOPEN_MODE_TORQUE:
    case CANOPEN_MODE_CST:
      cmd->control_mode = CONTROL_MODE_TORQUE;
      cmd->iq_ref = (float)target;
      break;
    default:
      return PARSE_UNKNOWN_ID;
    }

    return PARSE_OK;
  }

  /* SDO 下载 (最小化解析，仅用于参数读写扩展) */
  if (frame->id == (CANOPEN_FC_SDO_RX + s_node_id)) {
    if (frame->dlc < 4) {
      return PARSE_ERR_INVALID_FRAME;
    }

    uint16_t index = (uint16_t)(frame->data[1] << 8 | frame->data[0]);
    uint8_t subindex = frame->data[2];

    (void)subindex; // 当前实现不区分子索引

    if (index == CANOPEN_OBJ_CONTROLWORD) {
      cmd->has_control_word = true;
      cmd->control_word = BufToU16(&frame->data[4]);
      return PARSE_OK;
    }
    if (index == CANOPEN_OBJ_MODES_OF_OP) {
      s_current_mode = (CANopenMode)frame->data[4];
      return PARSE_OK;
    }
    if (index == CANOPEN_OBJ_TARGET_POSITION) {
      cmd->control_mode = CONTROL_MODE_POSITION;
      cmd->position_ref = (float)BufToI32(&frame->data[4]);
      return PARSE_OK;
    }
    if (index == CANOPEN_OBJ_TARGET_VELOCITY) {
      cmd->control_mode = CONTROL_MODE_VELOCITY;
      cmd->speed_ref = (float)BufToI32(&frame->data[4]);
      return PARSE_OK;
    }
    if (index == CANOPEN_OBJ_TARGET_TORQUE) {
      cmd->control_mode = CONTROL_MODE_TORQUE;
      cmd->iq_ref = (float)BufToI32(&frame->data[4]);
      return PARSE_OK;
    }

    return PARSE_UNKNOWN_ID;
  }

  return PARSE_UNKNOWN_ID;
}

bool ProtocolCANopen_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }

  frame->id = CANOPEN_FC_TPDO1 + s_node_id;
  frame->is_extended = false;
  frame->is_rtr = false;
  frame->dlc = 8;

  int32_t pos = (int32_t)status->position;
  int32_t vel = (int32_t)status->velocity;

  frame->data[0] = (uint8_t)(pos & 0xFF);
  frame->data[1] = (uint8_t)((pos >> 8) & 0xFF);
  frame->data[2] = (uint8_t)((pos >> 16) & 0xFF);
  frame->data[3] = (uint8_t)((pos >> 24) & 0xFF);

  frame->data[4] = (uint8_t)(vel & 0xFF);
  frame->data[5] = (uint8_t)((vel >> 8) & 0xFF);
  frame->data[6] = (uint8_t)((vel >> 16) & 0xFF);
  frame->data[7] = (uint8_t)((vel >> 24) & 0xFF);

  return true;
}

bool ProtocolCANopen_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  frame->id = CANOPEN_FC_EMCY + s_node_id;
  frame->is_extended = false;
  frame->is_rtr = false;
  frame->dlc = 8;

  frame->data[0] = (uint8_t)(fault_code & 0xFF);
  frame->data[1] = (uint8_t)((fault_code >> 8) & 0xFF);
  frame->data[2] = (uint8_t)((fault_code >> 16) & 0xFF);
  frame->data[3] = (uint8_t)((fault_code >> 24) & 0xFF);
  memset(&frame->data[4], 0, 4);

  return true;
}

bool ProtocolCANopen_HandleNMT(const CAN_Frame *frame) {
  if (frame == NULL || frame->dlc < 2) {
    return false;
  }

  uint8_t cmd = frame->data[0];
  uint8_t target = frame->data[1];
  if (target != 0 && target != s_node_id) {
    return false;
  }

  switch (cmd) {
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
  if (frame == NULL) {
    return false;
  }

  if (now_ms - s_last_hb_ms < 1000U) {
    return false;
  }

  s_last_hb_ms = now_ms;

  frame->id = CANOPEN_FC_HEARTBEAT + s_node_id;
  frame->is_extended = false;
  frame->is_rtr = false;
  frame->dlc = 1;
  frame->data[0] = (uint8_t)s_node_state;
  return true;
}
