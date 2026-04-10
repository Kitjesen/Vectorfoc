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
 * @file task_comm.c
 * @brief  - CANstate (500Hz)
 */
#include "FreeRTOS.h"
#include "cmd_service.h"
#include "cmsis_os.h"
#include "manager.h"
#include "rtos_tasks.h"
__attribute__((noreturn)) void StartCustomTask(void const *argument) {
  (void)argument;
  CmdService_Init();
  for (;;) {
    Protocol_ProcessQueuedFrames();
    CmdService_Process();
    osDelay(2);
  }
}
