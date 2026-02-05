/**
 * @file safety_control.h
 * @brief 安全控制层 - 集成统一错误管理器
 * @version 4.0
 * @date 2026-01-28
 */

#ifndef SAFETY_CONTROL_H
#define SAFETY_CONTROL_H

#include "../../error/error_manager.h"
#include "fault_detection.h"
#include "fsm.h"
#include "motor.h"

/* ========== 核心API ========== */

/**
 * @brief 初始化安全控制系统
 * @param config 配置参数（NULL=使用默认配置）
 */
void Safety_Init(const SafetyConfig *config);

/**
 * @brief 安全监控主函数（周期调用，如20kHz）
 * @param motor 电机数据指针
 * @param fsm 状态机指针
 */
void Safety_Update(MOTOR_DATA *motor, StateMachine *fsm);

/**
 * @brief 快速安全检测 (20kHz)
 * @param motor 电机数据指针
 * @param fsm 状态机指针
 * @note 仅检测过流等致命故障，CPU开销约1μs
 */
void Safety_Update_Fast(MOTOR_DATA *motor, StateMachine *fsm);

/**
 * @brief 慢速安全检测 (200Hz)
 * @param motor 电机数据指针
 * @param fsm 状态机指针
 * @note 检测缓变故障，CPU开销约3μs
 */
void Safety_Update_Slow(MOTOR_DATA *motor, StateMachine *fsm);

/**
 * @brief 清除故障状态
 * @param fsm 状态机指针
 */
void Safety_ClearFaults(StateMachine *fsm);

/**
 * @brief 检查是否有激活的故障
 * @return true=有故障，false=无故障
 */
bool Safety_HasActiveFault(void);

/**
 * @brief 获取当前激活的故障位掩码（FaultBit格式）
 * @return 故障位掩码
 */
uint32_t Safety_GetActiveFaultBits(void);

/**
 * @brief 注册故障回调函数
 * @param callback 回调函数指针
 */
void Safety_RegisterFaultCallback(SafetyFaultCallback callback);

/**
 * @brief 获取最后一次故障发生的时间戳
 * @return 故障时间戳 (ms, from HAL_GetTick)
 */
uint32_t Safety_GetLastFaultTime(void);

#endif /* SAFETY_CONTROL_H */
