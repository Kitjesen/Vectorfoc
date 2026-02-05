/**
 * @file fault_detection.h
 * @brief 故障检测层 - 纯物理量检测算法
 * @version 2.0
 * @date 2026-01-23
 */

#ifndef FAULT_DETECTION_H
#define FAULT_DETECTION_H

#include "fault_def.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化检测模块
 * @param config 配置参数指针 (NULL使用默认值)
 */
void Detection_Init(const DetectionConfig *config);

/**
 * @brief 执行单次检测 (无状态机逻辑)
 * @param motor 电机数据指针 (void* 避免循环依赖)
 * @return 故障码位掩码
 */
uint32_t Detection_Check(void *motor);

/**
 * @brief 快速故障检测 (仅检测致命故障，20kHz调用)
 * @param motor 电机数据指针
 * @return 故障码位掩码
 * @note 仅检测过流等紧急故障，执行时间约1μs
 */
uint32_t Detection_Check_Fast(void *motor);

/**
 * @brief 慢速故障检测 (检测缓变故障，200Hz调用)
 * @param motor 电机数据指针
 * @return 故障码位掩码
 * @note 检测电压、温度、堵转、CAN超时等，执行时间约3μs
 */
uint32_t Detection_Check_Slow(void *motor);

/**
 * @brief 重置检测状态 (如清零计数器)
 */
void Detection_Reset(void);

/**
 * @brief 获取当前检测状态 (只读)
 */
const DetectionState *Detection_GetState(void);

/**
 * @brief 更新CAN活动时间 (用于喂狗)
 */
void Detection_FeedWatchdog(uint32_t timestamp);

/**
 * @brief 获取当前配置 (用于参数读写)
 * @return 配置指针 (可修改)
 */
DetectionConfig *Detection_GetConfig(void);

/**
 * @brief 直接访问配置结构体 (用于参数表静态初始化)
 */
extern DetectionConfig s_config;

#endif /* FAULT_DETECTION_H */
