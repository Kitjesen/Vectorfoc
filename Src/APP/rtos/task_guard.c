/**
 * @file task_guard.c
 * @brief 保护监控任务 - 电机安全检测和LED指示 (200Hz)
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "led.h"
#include "motor.h"
#include "safety_control.h"
#include "rtos_tasks.h"


__attribute__((noreturn)) void StartGuardTask(void const *argument) {
  (void)argument;

  for (;;) {
    MotorGuardTask(&motor_data);
    osDelay(5);
  }
}
