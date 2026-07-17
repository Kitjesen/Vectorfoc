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
 * @file task_guard.c
 * @brief 200 Hz safety supervision and periodic diagnostic logging.
 */
#include "FreeRTOS.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "motor.h"
#include "rtos_tasks.h"
#include "safety_control.h"
#include "../../SAFE/watchdog_supervisor.h"

__attribute__((noreturn)) void StartGuardTask(void const *argument) {
  (void)argument;
  uint32_t diagnostic_count = 0u;
  WatchdogSupervisorState watchdog_state;
  WatchdogSupervisor_Init(
      &watchdog_state, HAL_GetSystemTick(), WATCHDOG_SUPERVISION_WINDOW_MS,
      WatchdogSupervisor_GetFOCHeartbeat(),
      WatchdogSupervisor_GetCommHeartbeat());

  for (;;) {
    /* This task observes and reports safety state.  It must never change the
     * requested control mode, clear faults, or request motor enable. */
    MotorGuardTask(&motor_data);

    WatchdogWindowStatus watchdog_status = WatchdogSupervisor_Evaluate(
        &watchdog_state, HAL_GetSystemTick(),
        WatchdogSupervisor_GetFOCHeartbeat(),
        WatchdogSupervisor_GetCommHeartbeat());
    if (watchdog_status == WATCHDOG_WINDOW_HEALTHY) {
      HAL_WatchdogFeed();
    }

    if (++diagnostic_count >= 200u) {
      diagnostic_count = 0u;
      uint32_t faults = Safety_GetActiveFaultBits();

      LOGINFO("FSM=%d mode=%d fault=%08X ctrl=%d",
              (int)StateMachine_GetState(&g_ds402_state_machine),
              (int)motor_data.state.State_Mode, (unsigned int)faults,
              (int)motor_data.state.Control_Mode);
    }

    osDelay(5u);
  }
}
