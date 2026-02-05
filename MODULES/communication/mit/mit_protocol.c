/**
 * @file protocol_mit.c
 * @brief MIT Cheetah协议实现
 * @version 1.0
 * @date 2026-01-20
 */

#include "mit_protocol.h"
#include "motor.h" // For g_can_id
#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint8_t s_motor_can_id = 1;

/**
 * @brief 将uint转为float (16bit定点数)
 */
static float Uint16ToFloat(uint16_t x, float min_val, float max_val) {
  float span = max_val - min_val;
  return ((float)x / 65535.0f) * span + min_val;
}

static uint16_t FloatToUint16(float x, float min_val, float max_val) {
  float span = max_val - min_val;
  float scaled = (x - min_val) / span;
  if (scaled < 0.0f)
    scaled = 0.0f;
  if (scaled > 1.0f)
    scaled = 1.0f;
  return (uint16_t)(scaled * 65535.0f);
}

static float Uint12ToFloat(uint16_t x, float min_val, float max_val) {
  float span = max_val - min_val;
  return ((float)x / 4095.0f) * span + min_val;
}

/**
 * @brief 将float转为uint (12bit定点数)
 */
static uint16_t FloatToUint12(float x, float min_val, float max_val) {
  float span = max_val - min_val;
  float scaled = (x - min_val) / span;
  if (scaled < 0.0f)
    scaled = 0.0f;
  if (scaled > 1.0f)
    scaled = 1.0f;
  return (uint16_t)(scaled * 4095.0f);
}

/**
 * @brief 初始化MIT协议
 */
void ProtocolMIT_Init(void) { s_motor_can_id = 1; }

/**
 * @brief 解析MIT运控命令
 * MIT Cheetah格式 (8字节):
 * [Position(16bit)][Velocity(12bit)][Kp(12bit)][Kd(12bit)][Torque(12bit)]
 * 共: 16+12+12+12+12 = 64 bits = 8 bytes
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

  cmd->control_mode = 2; // MIT运控模式
  cmd->enable_motor = true;

  return PARSE_OK;
}

/**
 * @brief 解析MIT协议帧
 */
ParseResult ProtocolMIT_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    return PARSE_ERR_INVALID_FRAME;
  }

  memset(cmd, 0, sizeof(MotorCommand));

  // MIT协议通过CAN ID区分命令类型
  // 标准CAN ID格式: 0x00-0x7F为电机ID
  // 扩展ID格式可包含命令类型

  // 简化处理: 根据DLC判断
  if (frame->dlc == 8) {
    // 8字节为运控命令
    return ParseMITControl(frame, cmd);
  } else if (frame->dlc == 0) {
    // 0字节命令 (通过ID区分)
    if (frame->is_rtr) {
      // RTR帧: 请求状态
      return PARSE_OK;
    }
  }

  return PARSE_ERR_UNSUPPORTED;
}

/**
 * @brief 构建MIT反馈帧
 * MIT反馈格式 (6字节):
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
 * @brief 构建MIT故障帧
 */
bool ProtocolMIT_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  frame->id = g_can_id;
  frame->dlc = 4;
  frame->is_extended = false;
  frame->is_rtr = false;

  // 使用简化的故障帧格式
  frame->data[0] = 0xFF; // 故障标识
  frame->data[1] = (fault_code >> 16) & 0xFF;
  frame->data[2] = (fault_code >> 8) & 0xFF;
  frame->data[3] = fault_code & 0xFF;

  return true;
}
