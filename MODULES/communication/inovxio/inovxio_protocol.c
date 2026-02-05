#include "inovxio_protocol.h"
#include "fault_def.h"
#include "manager.h" // For Protocol_SendFrame
#include "motor.h"
#include "param_access.h"
#include "protocol_types.h"
#include "rtos/cmd_service.h" // For CmdService_SetReportEnable
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ========== 辅助宏定义 ==========
// ID结构: [28:24] Cmd, [23:8] Data/MasterID, [7:0] TargetID
#define GET_CMD_TYPE(id) (((id) >> 24) & 0x1F)
#define GET_TARGET_ID(id) ((id) & 0xFF)
#define GET_ID_DATA(id) (((id) >> 8) & 0xFFFF)

#define BOOTLOADER_ADDR 0x1FFF0000U

static void JumpToBootloader(void) {
  uint32_t sp = *(__IO uint32_t *)(BOOTLOADER_ADDR);
  uint32_t reset = *(__IO uint32_t *)(BOOTLOADER_ADDR + 4U);

  if (sp < 0x20000000U) {
    return;
  }

  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  for (uint32_t i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  __set_MSP(sp);
  __DSB();
  __ISB();
  ((void (*)(void))reset)();
}

// ========== 数据转换辅助函数 ==========

/**
 * @brief Uint16 转 Float (线性映射)
 */
static float Uint16ToFloat(uint16_t x, float min_val, float max_val) {
  float span = max_val - min_val;
  return ((float)x / 65535.0f) * span + min_val;
}

/**
 * @brief Float 转 Uint16 (线性映射)
 */
static uint16_t FloatToUint16(float x, float min_val, float max_val) {
  float span = max_val - min_val;
  if (x < min_val)
    x = min_val;
  if (x > max_val)
    x = max_val;
  return (uint16_t)(((x - min_val) / span) * 65535.0f);
}

/**
 * @brief 16位大端解包 (高字节在前, 低字节在后)
 */
static uint16_t BufToUint16(const uint8_t *val_ptr) {
  return (uint16_t)((val_ptr[0] << 8) | val_ptr[1]);
}

/**
 * @brief 16位大端打包
 */
static void Uint16ToBuf(uint16_t val, uint8_t *val_ptr) {
  val_ptr[0] = (val >> 8) & 0xFF;
  val_ptr[1] = val & 0xFF;
}

/**
 * @brief 解析运控模式控制帧 (命令 1)
 * @details ID Data: 力矩 (0~65535 -> -120~120Nm)
 *          Payload: [PosH][PosL][VelH][VelL][KpH][KpL][KdH][KdL]
 */
static ParseResult ParseMotorCtrl(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame->dlc < 8)
    return PARSE_ERR_INVALID_FRAME;

  // 1. 从 ID 解析力矩
  uint16_t torque_raw = GET_ID_DATA(frame->id);
  cmd->torque_ff = Uint16ToFloat(torque_raw, -120.0f, 120.0f);

  // 2. 从数据段解析参数 (大端序)
  uint16_t pos_raw = BufToUint16(&frame->data[0]);
  uint16_t vel_raw = BufToUint16(&frame->data[2]);
  uint16_t kp_raw = BufToUint16(&frame->data[4]);
  uint16_t kd_raw = BufToUint16(&frame->data[6]);

  // 3. 转换为物理量
  cmd->pos_setpoint = Uint16ToFloat(pos_raw, -12.57f, 12.57f); // Pos: ±4π rad
  cmd->vel_setpoint = Uint16ToFloat(vel_raw, -15.0f, 15.0f); // Vel: ±15.0 rad/s
  cmd->kp = Uint16ToFloat(kp_raw, 0.0f, 500.0f); // Kp: 0~500 (高刚度)
  cmd->kd = Uint16ToFloat(kd_raw, 0.0f, 100.0f); // Kd: 0~100

  cmd->control_mode = 6; // MIT 阻抗模式
  cmd->enable_motor = true;

  return PARSE_OK;
}

/**
 * @brief 解析参数读写命令 (命令 17/18)
 * @details 数据格式: [IdxL][IdxH][Pad][Pad][Val0][Val1][Val2][Val3]
 */
static ParseResult ParseParamCommand(const CAN_Frame *frame, MotorCommand *cmd,
                                     bool is_write) {
  if (frame->dlc < 4)
    return PARSE_ERR_INVALID_FRAME;

  // 参数索引 (小端序)
  cmd->param_index = (uint16_t)(frame->data[0] | (frame->data[1] << 8));

  if (is_write) {
    if (frame->dlc < 8)
      return PARSE_ERR_INVALID_FRAME;
    // 参数值 (4 字节)
    memcpy(&cmd->param_value, &frame->data[4], 4);
    cmd->is_param_write = true;
  } else {
    cmd->is_param_read = true;
  }

  return PARSE_OK;
}

/**
 * @brief 协议解析入口函数
 */
ParseResult ProtocolPrivate_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (!frame || !cmd)
    return PARSE_ERR_INVALID_FRAME;

  // 1. 检查是否为扩展帧
  if (!frame->is_extended)
    return PARSE_ERR_INVALID_FRAME;

  memset(cmd, 0, sizeof(MotorCommand));
  PrivateCmdType type = (PrivateCmdType)GET_CMD_TYPE(frame->id);

  switch (type) {
  case PRIVATE_CMD_MOTOR_CTRL: // 1
    return ParseMotorCtrl(frame, cmd);

  case PRIVATE_CMD_MOTOR_ENABLE: // 3
    cmd->enable_motor = true;
    return PARSE_OK;

  case PRIVATE_CMD_MOTOR_STOP: // 4
    cmd->enable_motor = false;
    return PARSE_OK;

  case PRIVATE_CMD_SET_ZERO: // 6
    cmd->set_zero = true;
    return PARSE_OK;

  case PRIVATE_CMD_CALIBRATE: // 8 (New)
    // 触发电机校准 (异步处理)
    {
      uint8_t type = 0;
      if (frame->dlc >= 1)
        type = frame->data[0];
      Motor_RequestCalibration(&motor_data, type);
    }
    return PARSE_OK;

  case PRIVATE_CMD_SET_ID: // 7
    // 直接在此处通过 Param API 修改 ID
    if (frame->dlc >= 4) { // Assumed layout: [NewID][0][0][0]
      // 但是ID通常只有一个字节?
      // 假设 MasterID 在字节1?
      // 简单处理: Byte0 = NewID
      float new_id = (float)frame->data[0];
      Param_WriteFloat(PARAM_CAN_ID, new_id);
      Param_ScheduleSave(); // 延迟保存（ISR安全）
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;

  case PRIVATE_CMD_PARAM_READ: // 17
    return ParseParamCommand(frame, cmd, false);

  case PRIVATE_CMD_PARAM_WRITE: // 18
    return ParseParamCommand(frame, cmd, true);

  case PRIVATE_CMD_SAVE: // 22
    Param_ScheduleSave();
    return PARSE_OK;

  case PRIVATE_CMD_GET_VERSION: // 26
    // 返回固件版本号
    // 响应: [Major][Minor][Patch][0][BuiltYear][Month][Day][0]
    // 示例: 1.0.0
    {
      CAN_Frame tx_frame;
      tx_frame.id = (0x1A << 24) | (g_can_id << 8) | 0xFD;
      tx_frame.is_extended = true;
      tx_frame.dlc = 8;
      tx_frame.data[0] = 1; // Major
      tx_frame.data[1] = 0; // Minor
      tx_frame.data[2] = 0; // Patch
      tx_frame.data[3] = 0;

      Protocol_SendFrame(&tx_frame);
    }
    return PARSE_OK;

  case PRIVATE_CMD_FAULT_QUERY: // 30
    cmd->is_fault_query = true;
    return PARSE_OK;

  case PRIVATE_CMD_RESET: // 11
    HAL_NVIC_SystemReset();
    return PARSE_OK;

  case PRIVATE_CMD_CLEAR_FAULT: // 12
    Motor_ClearFaults(&motor_data);
    return PARSE_OK;

  case PRIVATE_CMD_BOOTLOADER: // 13 - 跳转Bootloader
    JumpToBootloader();
    return PARSE_OK;

    // 以下命令暂未具体实现，但占位返回 OK 以免报错

  case PRIVATE_CMD_REPORT: // 24
    if (frame->dlc >= 1) {
      // 0: Off, 1: On
      CmdService_SetReportEnable(frame->data[0] != 0);
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;

  case PRIVATE_CMD_SET_BAUDRATE: // 23
  case PRIVATE_CMD_SET_PROTOCOL: // 25
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

/**
 * @brief 构建电机反馈帧 (命令 2)
 * @details ID结构: [Cmd(5)][Mode(2)+Fault(6)+MasterID(8)][MyID(8)][Host(8)]
 */
bool ProtocolPrivate_BuildFeedback(const MotorStatus *status,
                                   CAN_Frame *frame) {
  if (!status || !frame)
    return false;

  uint16_t master_id = 0xFD; // 默认主机ID

  // 构造ID中间部分 (Bit 23-8)
  // MinerU MD:
  // Bit 23-22: Mode (0:Reset, 1:Cali, 2:Motor)
  // Bit 21-16: Fault bits (6 bits)
  // Bit 15-8:  Master ID (0xFD)

  uint32_t mode = 2; // Motor Mode

  // 映射故障位 (Bit 21-16)
  // Bit 21: 未标定 (FAULT_ENCODER_UNCALIBRATED)
  // Bit 20: 堵转过载 (FAULT_STALL_OVERLOAD)
  // Bit 19: 磁编码故障 (FAULT_HARDWARE_ID like)
  // Bit 18: 过温 (FAULT_OVER_TEMP)
  // Bit 17: 三相电流故障 (FAULT_CURRENT_A/B/C)
  // Bit 16: 欠压故障 (FAULT_UNDER_VOLTAGE)

  uint32_t fault_bits = 0;
  if (status->fault_code & FAULT_ENCODER_UNCALIBRATED)
    fault_bits |= (1 << 5); // Bit 21
  if (status->fault_code & FAULT_STALL_OVERLOAD)
    fault_bits |= (1 << 4); // Bit 20
  if (status->fault_code & FAULT_HARDWARE_ID)
    fault_bits |= (1 << 3); // Bit 19
  if (status->fault_code & FAULT_OVER_TEMP)
    fault_bits |= (1 << 2); // Bit 18
  if (status->fault_code &
      (FAULT_CURRENT_A | FAULT_CURRENT_B | FAULT_CURRENT_C))
    fault_bits |= (1 << 1); // Bit 17
  if (status->fault_code & FAULT_UNDER_VOLTAGE)
    fault_bits |= (1 << 0); // Bit 16

  // ID构建: [Cmd 5bit][Info 16bit][Target 8bit]
  // Cmd: 0x02
  // Info: Mode(2) + Faults(6) + Master(8)
  // Target: HostID (0xFF/0xFD?) -> MD Feedback Frame defines Target as Host ID

  uint32_t info = (mode << 14) | (fault_bits << 8) |
                  0xFD; // 0xFD as MasterID in high byte of low word?

  uint32_t id = (0x02 << 24) | (mode << 22) | (fault_bits << 16) |
                (status->can_id << 8) | 0xFD; // Host ID

  frame->id = id;
  frame->is_extended = true;
  frame->dlc = 8;

  // ========== 数据域: [位置][速度][力矩][温度] ==========
  uint16_t pos = FloatToUint16(status->position, -12.57f, 12.57f);
  uint16_t vel = FloatToUint16(status->velocity, -15.0f, 15.0f);
  uint16_t tor = FloatToUint16(status->torque, -120.0f, 120.0f);
  int16_t temp = (int16_t)(status->temperature * 10.0f); // 温度 × 10

  Uint16ToBuf(pos, &frame->data[0]);
  Uint16ToBuf(vel, &frame->data[2]);
  Uint16ToBuf(tor, &frame->data[4]);
  Uint16ToBuf((uint16_t)temp, &frame->data[6]);

  return true;
}

/**
 * @brief 构建故障帧 (命令 21)
 */
bool ProtocolPrivate_BuildFault(uint32_t fault_code, uint32_t warning_code,
                                CAN_Frame *frame) {
  if (!frame)
    return false;

  // 命令 21: 故障上报
  frame->id = (0x15 << 24) | (g_can_id << 8) | 0xFD;
  frame->is_extended = true;
  frame->dlc = 8;

  // 故障代码 (4 字节, 大端序)
  frame->data[0] = (fault_code >> 24) & 0xFF;
  frame->data[1] = (fault_code >> 16) & 0xFF;
  frame->data[2] = (fault_code >> 8) & 0xFF;
  frame->data[3] = fault_code & 0xFF;

  return true;
}

/**
 * @brief 初始化 Inovxio 协议模块
 */
void ProtocolPrivate_Init(void) {
  // 预留初始化接口
}

/**
 * @brief 构建参数读取响应帧 (命令 18 响应)
 */
bool ProtocolPrivate_BuildParamResponse(uint16_t param_index, float value,
                                        CAN_Frame *frame) {
  if (!frame)
    return false;

  // 命令 18 响应: 参数读取结果
  frame->id = (0x12 << 24) | (g_can_id << 8) | 0xFD;
  frame->is_extended = true;
  frame->dlc = 8;

  // 参数索引 (小端序)
  frame->data[0] = param_index & 0xFF;
  frame->data[1] = (param_index >> 8) & 0xFF;
  frame->data[2] = 0;
  frame->data[3] = 0;

  // 参数值 (4 字节浮点数)
  memcpy(&frame->data[4], &value, 4);

  return true;
}

/**
 * @brief 构建故障详情帧 (命令 30)
 * @param status 电机状态
 * @param frame [out] CAN 帧
 * @return 成功返回 true, 失败返回 false
 */
bool ProtocolPrivate_BuildFaultDetail(const MotorStatus *status,
                                      CAN_Frame *frame) {
  if (!status || !frame)
    return false;

  // CMD 30响应: [CMD 30][SourceID][TargetID]
  frame->id = (0x1E << 24) | (status->can_id << 8) | 0xFD;
  frame->is_extended = true;
  frame->dlc = 8;

  // Data[0-1]: Fault Code (uint16, Big-Endian)
  frame->data[0] = (status->fault_code >> 8) & 0xFF;
  frame->data[1] = (status->fault_code) & 0xFF;

  // Data[2-3]: Fault Code High (Reserved, 0)
  frame->data[2] = (status->fault_code >> 24) & 0xFF;
  frame->data[3] = (status->fault_code >> 16) &
                   0xFF; // Not strictly big-endian logic for reserved, but
                         // consistent with 32-bit split

  // Data[4-7]: Fault Timestamp (uint32, Big-Endian)
  uint32_t timestamp = Safety_GetLastFaultTime();
  frame->data[4] = (timestamp >> 24) & 0xFF;
  frame->data[5] = (timestamp >> 16) & 0xFF;
  frame->data[6] = (timestamp >> 8) & 0xFF;
  frame->data[7] = timestamp & 0xFF;

  return true;
}
