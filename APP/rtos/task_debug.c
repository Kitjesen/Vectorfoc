/**
 * @file task_debug.c
 * @brief 调试任务 - USB/VOFA数据输出 (1kHz)
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "rtos_tasks.h"
#include "usb_device.h"
#include "vofa.h"


__attribute__((noreturn)) void StartDefaultTask(void const *argument) {
  (void)argument;
  MX_USB_Device_Init();

  for (;;) {
    Scope_Process();
    osDelay(1);
  }
}
