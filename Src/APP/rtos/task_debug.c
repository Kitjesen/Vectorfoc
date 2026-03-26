/**
 * @file task_debug.c
 * @brief / - USB output + VectorStudio state
 *
 * @details
 *   runningfrequency: 1kHz (osDelay(1))
 *   -  1ms:    (Scope_Process, FireWater )
 *   -  100ms: state (Studio_PeriodicUpdate, calibration/fault/)
 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "rtos_tasks.h"
#include "usb_device.h"
#include "vofa.h"
/** state (1kHz / 100 = 10Hz) */
#define STATUS_REPORT_DIVIDER 100
__attribute__((noreturn)) void StartDefaultTask(void const *argument) {
  (void)argument;
#ifndef BOARD_XSTAR
  /* USB CDC + VectorStudio are only available on VectorFOC board */
  MX_USB_Device_Init();
  uint16_t status_cnt = 0;
  for (;;) {
    Scope_Process();
    if (++status_cnt >= STATUS_REPORT_DIVIDER) {
      status_cnt = 0;
      Studio_PeriodicUpdate();
    }
    osDelay(1);
  }
#else
  /* BOARD_XSTAR has no USB; just yield forever */
  for (;;) {
    osDelay(1000);
  }
#endif
}
