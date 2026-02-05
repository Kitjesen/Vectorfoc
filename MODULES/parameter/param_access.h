/**
 * @file param_access.h
 * @brief 参数访问接口 - 统一的参数读写API
 *
 * 职责:
 *   - 提供类型安全的参数读写接口
 *   - 自动进行范围和权限验证
 *   - 管理Flash持久化存储
 *
 * 使用示例:
 *   // 读取参数
 *   float Rs;
 *   Param_ReadFloat(PARAM_MOTOR_RS, &Rs);
 *
 *   // 写入参数
 *   Param_WriteFloat(PARAM_MOTOR_RS, 0.8f);
 *
 *   // 持久化保存
 *   Param_SaveToFlash();
 */

#ifndef PARAM_ACCESS_H
#define PARAM_ACCESS_H

#include "param_table.h"

/**
 * @brief 读取参数值
 * @param index 参数索引
 * @param data 输出数据缓冲区
 * @param type 输出数据类型 (可为NULL)
 * @return ParamResult 操作结果
 */
ParamResult Param_Read(uint16_t index, void *data, ParamType *type);

/**
 * @brief 写入参数值
 * @param index 参数索引
 * @param data 输入数据缓冲区
 * @return ParamResult 操作结果
 */
ParamResult Param_Write(uint16_t index, const void *data);

/**
 * @brief 读取float类型参数 (便捷接口)
 * @param index 参数索引
 * @param value 输出值
 * @return ParamResult 操作结果
 */
ParamResult Param_ReadFloat(uint16_t index, float *value);

/**
 * @brief 写入float类型参数 (便捷接口)
 * @param index 参数索引
 * @param value 输入值
 * @return ParamResult 操作结果
 */
ParamResult Param_WriteFloat(uint16_t index, float value);

/**
 * @brief 读取uint8类型参数 (便捷接口)
 * @param index 参数索引
 * @param value 输出值
 * @return ParamResult 操作结果
 */
ParamResult Param_ReadUint8(uint16_t index, uint8_t *value);

/**
 * @brief 写入uint8类型参数 (便捷接口)
 * @param index 参数索引
 * @param value 输入值
 * @return ParamResult 操作结果
 */
ParamResult Param_WriteUint8(uint16_t index, uint8_t value);

/**
 * @brief 保存所有可存储参数到Flash
 * @return ParamResult 操作结果
 */
ParamResult Param_SaveToFlash(void);

/**
 * @brief 从Flash加载所有参数
 * @return ParamResult 操作结果
 */
ParamResult Param_LoadFromFlash(void);

/**
 * @brief 初始化参数系统（仅执行一次）
 * @return ParamResult 操作结果
 */
ParamResult Param_SystemInitOnce(void);

/**
 * @brief 恢复所有参数为默认值
 * @return ParamResult 操作结果
 */
ParamResult Param_RestoreDefaults(void);

/**
 * @brief 获取参数信息
 * @param index 参数索引
 * @param entry 输出参数条目指针
 * @return ParamResult 操作结果
 */
ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry);

/**
 * @brief 调度一次Flash保存 (非阻塞，ISR安全)
 * @note 仅设置标志位，实际保存由 Param_ProcessScheduledSave 在主循环执行
 */
void Param_ScheduleSave(void);

/**
 * @brief 处理调度的保存请求 (需在主循环调用)
 * @return true if save occurred, false otherwise
 */
bool Param_ProcessScheduledSave(void);

#endif /* PARAM_ACCESS_H */
