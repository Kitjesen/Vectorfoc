/**
 * @file hal_abstraction.h
 * @brief HAL抽象层 - 提供可移植的硬件接口
 * @version 1.0
 * @date 2026-01-20
 * 
 * 本文件抽象了硬件相关的函数调用，使代码可以在不同的HAL库之间移植
 */

#ifndef HAL_ABSTRACTION_H
#define HAL_ABSTRACTION_H

#include "stdint.h"
#include "stdbool.h"
#include "protocol_types.h"

/**
 * @brief 获取系统时钟滴答数（毫秒）
 * @return 系统启动以来的毫秒数
 * 
 * @note 在STM32 HAL中通常映射到HAL_GetTick()
 */
uint32_t HAL_GetSystemTick(void);

/**
 * @brief 获取微秒级时间戳（可选，用于高精度计时）
 * @return 系统启动以来的微秒数
 */
uint32_t HAL_GetMicroseconds(void);

/**
 * @brief 发送CAN帧
 * @param frame CAN帧指针
 * @return true=发送成功, false=发送失败（邮箱满或其他错误）
 * 
 * @note 此函数会自动检测CAN ID类型（标准/扩展）和RTR标志
 */
bool HAL_CAN_Transmit(const CAN_Frame *frame);

/**
 * @brief 检查CAN发送邮箱是否可用
 * @return true=有可用邮箱, false=所有邮箱已满
 */
bool HAL_CAN_IsTxMailboxAvailable(void);

/**
 * @brief 获取温度传感器读数
 * @return 温度值（摄氏度）
 * 
 * @note 如果没有硬件温度传感器，可以返回估算值
 */
float HAL_GetTemperature(void);

/**
 * @brief 延时函数（毫秒）
 * @param ms 延时毫秒数
 */
void HAL_Delay(uint32_t ms);

/**
 * @brief 进入临界区（禁用中断）
 * @return 之前的中断状态（用于恢复）
 */
uint32_t HAL_EnterCritical(void);

/**
 * @brief 退出临界区（恢复中断）
 * @param prev_state 之前的中断状态
 */
void HAL_ExitCritical(uint32_t prev_state);

#endif /* HAL_ABSTRACTION_H */
