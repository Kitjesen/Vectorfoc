// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
#include "board_config.h"  // HW_USB_ENABLED
#include "vofa.h"
#if HW_USB_ENABLED
#include "usb_device.h"
#endif
/** state (1kHz / 100 = 10Hz) */
#define STATUS_REPORT_DIVIDER 100
__attribute__((noreturn)) void StartDefaultTask(void const *argument) {
  (void)argument;
#if HW_USB_ENABLED
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
  /* 本板无 USB，仅保持任务存活 */
  for (;;) {
    osDelay(1000);
  }
#endif
}
