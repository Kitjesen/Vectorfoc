/**
 * @file protocol_canopen.c
 * @brief CANopen DS402 协议实现
 * @version 1.0
 * @date 2026-01-20
 */

#include "canopen_protocol.h"
#include "fsm.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* CANopen功能码 (COB-ID) */
#define CANOPEN_FC_NMT 0x000
#define CANOPEN_FC_SYNC 0x080
#define CANOPEN_FC_EMCY 0x080
#define CANOPEN_FC_TPDO1 0x180
#define CANOPEN_FC_RPDO1 0x200
#define CANOPEN_FC_TPDO2 0x280
#define CANOPEN_FC_RPDO2 0x300
#define CANOPEN_FC_TPDO3 0x380
#define CANOPEN_FC_RPDO3 0x400
#define CANOPEN_FC_TPDO4 0x480
#define CANOPEN_FC_RPDO4 0x500
#define CANOPEN_FC_SDO_TX 0x580
#define CANOPEN_FC_SDO_RX 0x600
#define CANOPEN_FC_HEARTBEAT 0x700

/* CANopen SDO命令字 */
#define SDO_CMD_DOWNLOAD_INIT 0x20 // 写入请求
#define SDO_CMD_DOWNLOAD_RESP 0x60 // 写入响应
#define SDO_CMD_UPLOAD_INIT 0x40   // 读取请求
#define SDO_CMD_UPLOAD_RESP 0x40   // 读取响应

/* NMT Commands */
#define CANOPEN_NMT_START 0x01
#define CANOPEN_NMT_STOP 0x02
#define CANOPEN_NMT_PREOP 0x80
#define CANOPEN_NMT_RESET_NODE 0x81
#define CANOPEN_NMT_RESET_COMM 0x82

/* 配置 */
static uint8_t s_node_id = 1;
static CANopenMode s_current_mode = CANOPEN_MODE_CST;
static CANopenNodeState s_node_state = CANOPEN_STATE_PREOPERATIONAL;
static uint32_t s_hb_period_ms = 1000;
static uint32_t s_last_hb_ms = 0;

/**
 * @brief 从CAN ID提取功能码和节点ID
 */
static void DecodeCOB_ID(uint32_t can_id, uint16_t *func_code,
                         uint8_t *node_id) {
  *func_code = can_id & 0x780;
  *node_id = can_id & 0x7F;
}

/**
 * @brief 初始化CANopen协议
 */
void ProtocolCANopen_Init(void) {
  s_node_id = 1; // 可从配置加载
  s_current_mode = CANOPEN_MODE_CST;
  s_node_state = CANOPEN_STATE_PREOPERATIONAL;
  s_last_hb_ms = 0;
}

/**
 * @brief 解析RPDO1 (Receive PDO1) - 控制字
 */
static ParseResult ParseRPDO1(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame->dlc < 2) {
    return PARSE_ERR_INVALID_FRAME;
  }

  // Controlword (0x6040)
  uint16_t controlword = frame->data[0] | (frame->data[1] << 8);

  // Store raw controlword for FSM
  cmd->has_control_word = true;
  cmd->control_word = controlword;

  // 将controlword转换为电机命令
  // DS402标准位定义:
  // Bit 0: Switch On
  // Bit 1: Enable Voltage
  // Bit 2: Quick Stop
  // Bit 3: Enable Operation
  // Bit 7: Fault Reset

  if (controlword & (1 << 3)) {
    cmd->enable_motor = true;
  } else {
    cmd->enable_motor = false;
  }

  // 提取目标值 (取决于运行模式)
  if (frame->dlc >= 4) {
    int16_t target = (int16_t)(frame->data[2] | (frame->data[3] << 8));

    switch (s_current_mode) {
    case CANOPEN_MODE_CST:                      // 循环同步转矩
      cmd->torque_ff = (float)target / 1000.0f; // mNm -> Nm
      cmd->control_mode = 0;                    // 电流模式
      break;

    case CANOPEN_MODE_CSV:                      // 循环同步速度
      cmd->speed_ref = (float)target / 1000.0f; // mrad/s -> rad/s
      cmd->control_mode = 1;                    // 速度模式
      break;

    case CANOPEN_MODE_CSP:                          // 循环同步位置
      cmd->position_ref = (float)target / 10000.0f; // 0.1 mrad -> rad
      cmd->control_mode = 3;                        // 位置模式
      break;

    default:
      break;
    }
  }

  return PARSE_OK;
}

/**
 * @brief 解析SDO (Service Data Object)
 */
static ParseResult ParseSDO(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame->dlc < 8) {
    return PARSE_ERR_INVALID_FRAME;
  }

  uint8_t cmd_byte = frame->data[0];
  uint16_t index = frame->data[1] | (frame->data[2] << 8);
  uint8_t subindex = frame->data[3];

  (void)subindex; // 暂未使用

  // 处理写入命令
  if ((cmd_byte & 0xE0) == SDO_CMD_DOWNLOAD_INIT) {
    uint32_t value = frame->data[4] | (frame->data[5] << 8) |
                     (frame->data[6] << 16) | (frame->data[7] << 24);

    switch (index) {
    case CANOPEN_OBJ_CONTROLWORD:
      // 通过RPDO处理控制字
      break;

    case CANOPEN_OBJ_MODES_OF_OP:
      s_current_mode = (CANopenMode)(value & 0xFF);
      break;

    case CANOPEN_OBJ_TARGET_TORQUE:
      cmd->torque_ff = (float)((int16_t)value) / 1000.0f;
      cmd->control_mode = 0;
      break;

    case CANOPEN_OBJ_TARGET_VELOCITY:
      cmd->speed_ref = (float)((int32_t)value) / 1000.0f;
      cmd->control_mode = 1;
      break;

    case CANOPEN_OBJ_TARGET_POSITION:
      cmd->position_ref = (float)((int32_t)value) / 10000.0f;
      cmd->control_mode = 3;
      break;

    default:
      return PARSE_ERR_UNSUPPORTED;
    }

    return PARSE_OK;
  }

  // 处理读取命令
  if ((cmd_byte & 0xE0) == SDO_CMD_UPLOAD_INIT) {
    cmd->is_param_read = true;
    cmd->param_index = index;
    return PARSE_OK;
  }

  return PARSE_ERR_UNSUPPORTED;
}

/**
 * @brief 解析CANopen帧
 */
ParseResult ProtocolCANopen_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    return PARSE_ERR_INVALID_FRAME;
  }

  memset(cmd, 0, sizeof(MotorCommand));

  if (frame->id == CANOPEN_FC_NMT) {
    return ProtocolCANopen_HandleNMT(frame) ? PARSE_OK : PARSE_UNKNOWN_ID;
  }

  uint16_t func_code;
  uint8_t node_id;
  DecodeCOB_ID(frame->id, &func_code, &node_id);

  // 检查节点ID
  if (node_id != s_node_id) {
    return PARSE_ERR_INVALID_FRAME;
  }

  switch (func_code) {
  case CANOPEN_FC_RPDO1:
  case CANOPEN_FC_RPDO2:
  case CANOPEN_FC_RPDO3:
  case CANOPEN_FC_RPDO4:
    return ParseRPDO1(frame, cmd);

  case CANOPEN_FC_SDO_RX:
    return ParseSDO(frame, cmd);

  default:
    return PARSE_ERR_UNSUPPORTED;
  }
}

/**
 * @brief 构建TPDO1 (Transmit PDO1) - 状态反馈
 */
bool ProtocolCANopen_BuildFeedback(const MotorStatus *status,
                                   CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }

  frame->id = CANOPEN_FC_TPDO1 + s_node_id;
  frame->dlc = 8;
  frame->is_extended = false;
  frame->is_rtr = false;

  // Statusword (0x6041)
  uint16_t statusword = status->motor_state; // 实际应从state machine获取
  frame->data[0] = statusword & 0xFF;
  frame->data[1] = (statusword >> 8) & 0xFF;

  // Actual value (根据运行模式)
  int32_t actual_value = 0;
  switch (s_current_mode) {
  case CANOPEN_MODE_CST:                                // 实际转矩
    actual_value = (int32_t)(status->torque * 1000.0f); // Nm -> mNm
    break;

  case CANOPEN_MODE_CSV:                                  // 实际速度
    actual_value = (int32_t)(status->velocity * 1000.0f); // rad/s -> mrad/s
    break;

  case CANOPEN_MODE_CSP:                                   // 实际位置
    actual_value = (int32_t)(status->position * 10000.0f); // rad -> 0.1 mrad
    break;

  default:
    break;
  }

  frame->data[2] = actual_value & 0xFF;
  frame->data[3] = (actual_value >> 8) & 0xFF;
  frame->data[4] = (actual_value >> 16) & 0xFF;
  frame->data[5] = (actual_value >> 24) & 0xFF;

  // 额外信息: 温度和电压
  frame->data[6] = (uint8_t)status->temperature;
  frame->data[7] = (uint8_t)(status->voltage / 0.5f); // 0-127.5V, 0.5V步进

  return true;
}

/**
 * @brief 构建EMCY紧急帧
 */
bool ProtocolCANopen_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  frame->id = CANOPEN_FC_EMCY + s_node_id;
  frame->dlc = 8;
  frame->is_extended = false;
  frame->is_rtr = false;

  // Emergency Error Code (2字节)
  frame->data[0] = fault_code & 0xFF;
  frame->data[1] = (fault_code >> 8) & 0xFF;

  // Error Register (1字节)
  frame->data[2] = 0x01; // Generic error

  // Manufacturer specific error field (5字节)
  frame->data[3] = (fault_code >> 16) & 0xFF;
  frame->data[4] = (fault_code >> 24) & 0xFF;
  frame->data[5] = 0;
  frame->data[6] = 0;
  frame->data[7] = 0;

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
  case CANOPEN_NMT_PREOP:
    s_node_state = CANOPEN_STATE_PREOPERATIONAL;
    break;
  case CANOPEN_NMT_RESET_NODE:
    HAL_NVIC_SystemReset();
    break;
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

  if (s_hb_period_ms == 0) {
    return false;
  }

  if ((now_ms - s_last_hb_ms) < s_hb_period_ms) {
    return false;
  }

  s_last_hb_ms = now_ms;
  frame->id = CANOPEN_FC_HEARTBEAT + s_node_id;
  frame->dlc = 1;
  frame->is_extended = false;
  frame->is_rtr = false;
  frame->data[0] = (uint8_t)s_node_state;
  return true;
}
