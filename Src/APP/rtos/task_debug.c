/**
 * @file task_debug.c
 * @brief 调试任务 - USB/VOFA数据输出 (1kHz)
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "rtos_tasks.h"
#include "usb_device.h"
#include "vofa.h"

#define STATUS_REPORT_DIVIDER 100U

__attribute__((noreturn)) void StartDefaultTask(void const *argument) {
  uint16_t status_cnt = 0;

  (void)argument;
  MX_USB_Device_Init();
  Scope_Init();

  for (;;) {
    Vofa_Process();
    if (++status_cnt >= STATUS_REPORT_DIVIDER) {
      status_cnt = 0;
      Studio_PeriodicUpdate();
    }
    osDelay(1);
  }
}
