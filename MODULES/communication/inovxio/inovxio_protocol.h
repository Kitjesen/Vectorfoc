/**
 * @file    inovxio_protocol.h
 * @brief   Inovxio (MinerU/Robstride) 私有协议
 * @details 高性能运动控制协议
 *          单位: 位置 [rad], 速度 [rad/s], 力矩 [Nm], 电流 [A]
 * @version 1.0
 */

#ifndef PROTOCOL_INOVXIO_H
#define PROTOCOL_INOVXIO_H

#include "protocol_types.h"

/**
 * @brief Inovxio 协议命令类型
 */
typedef enum {
  PRIVATE_CMD_GET_ID = 0,         /**< 获取设备ID */
  PRIVATE_CMD_MOTOR_CTRL = 1,     /**< 运动控制命令 */
  PRIVATE_CMD_MOTOR_FEEDBACK = 2, /**< 电机反馈 */
  PRIVATE_CMD_MOTOR_ENABLE = 3,   /**< 电机使能 */
  PRIVATE_CMD_MOTOR_STOP = 4,     /**< 电机停止 */
  PRIVATE_CMD_SET_ZERO = 6,       /**< 设置零点 */
  PRIVATE_CMD_SET_ID = 7,         /**< 设置CAN ID */
  PRIVATE_CMD_CALIBRATE = 8,      /**< 触发校准 */
  PRIVATE_CMD_RESET = 0x0B,       /**< 系统复位 (11) */
  PRIVATE_CMD_CLEAR_FAULT = 0x0C, /**< 清除故障 (12) */
  PRIVATE_CMD_BOOTLOADER = 0x0D,  /**< 跳转到Bootloader (13) */
  PRIVATE_CMD_PARAM_READ = 17,    /**< 参数读取 */
  PRIVATE_CMD_PARAM_WRITE = 18,   /**< 参数写入 */
  PRIVATE_CMD_FAULT = 21,         /**< 故障反馈 */
  PRIVATE_CMD_SAVE = 22,          /**< 保存参数 */
  PRIVATE_CMD_SET_BAUDRATE = 23,  /**< 设置波特率 */
  PRIVATE_CMD_REPORT = 24,        /**< 主动上报 */
  PRIVATE_CMD_SET_PROTOCOL = 25,  /**< 切换协议 */
  PRIVATE_CMD_GET_VERSION = 26,   /**< 读取版本 */
  PRIVATE_CMD_FAULT_QUERY = 30,   /**< 故障详情查询 */
} PrivateCmdType;

/**
 * @brief  初始化 Inovxio 协议模块
 */
void ProtocolPrivate_Init(void);

/**
 * @brief  解析 Inovxio 协议 CAN 帧
 * @param  frame CAN 帧
 * @param  cmd   [out] 解析后的命令
 * @return 解析结果
 */
ParseResult ProtocolPrivate_Parse(const CAN_Frame *frame, MotorCommand *cmd);

/**
 * @brief  构建反馈帧
 * @param  status 电机状态
 * @param  frame  [out] CAN 帧
 * @return 成功返回 true, 失败返回 false
 */
bool ProtocolPrivate_BuildFeedback(const MotorStatus *status, CAN_Frame *frame);

/**
 * @brief  构建故障帧
 * @param  fault_code   故障代码
 * @param  warning_code 警告代码
 * @param  frame        [out] CAN 帧
 * @return 成功返回 true, 失败返回 false
 */
bool ProtocolPrivate_BuildFault(uint32_t fault_code, uint32_t warning_code,
                                CAN_Frame *frame);

/**
 * @brief  构建参数读取响应帧
 * @param  param_index 参数索引
 * @param  value       参数值
 * @param  frame       [out] CAN 帧
 * @return 成功返回 true, 失败返回 false
 */
bool ProtocolPrivate_BuildParamResponse(uint16_t param_index, float value,
                                        CAN_Frame *frame);

/**
 * @brief  构建故障详情帧 (CMD 30)
 * @param  status 电机状态
 * @param  frame  [out] CAN 帧
 * @return 成功返回 true, 失败返回 false
 */
bool ProtocolPrivate_BuildFaultDetail(const MotorStatus *status,
                                      CAN_Frame *frame);

#endif /* PROTOCOL_INOVXIO_H */
