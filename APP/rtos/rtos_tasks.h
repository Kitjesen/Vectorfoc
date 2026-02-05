/**
 * @file rtos_tasks.h
 * @brief RTOS任务接口声明
 */

#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 调试任务 - USB/VOFA (1kHz)
 */
void StartDefaultTask(void const *argument);

/**
 * @brief 保护任务 - 安全监控 (200Hz)
 */
void StartGuardTask(void const *argument);

/**
 * @brief 通信任务 - CAN通信 (500Hz)
 */
void StartCustomTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_TASKS_H */
