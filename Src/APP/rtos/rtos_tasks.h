/**
 * @file rtos_tasks.h
 * @brief RTOS
 */
#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief  - USB/VOFA (1kHz)
 */
void StartDefaultTask(void const *argument);
/**
 * @brief protection - safety (200Hz)
 */
void StartGuardTask(void const *argument);
/**
 * @brief  - CAN (500Hz)
 */
void StartCustomTask(void const *argument);
#ifdef __cplusplus
}
#endif
#endif /* RTOS_TASKS_H */
