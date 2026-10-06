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
 * @file safety_task.c
 * @brief protection - motorsafetyLED (200Hz)
 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "status_led.h"
#include "motor_runtime.h"
#include "safety_manager.h"
#include "application.h"
#include "protocol_dispatcher.h"
#include "protocol_messages.h"
#include "drive_state_machine.h"
#include "board_log.h"

__attribute__((noreturn)) void StartGuardTask(void const *argument) {
  (void)argument;
  uint32_t heartbeat_cnt = 0;

  for (;;) {
    MotorGuardTask(&motor_data);

    /* 每 200 次 × 5ms = 1s，发一帧心跳 CAN 帧（cmd=0x1F, target=0xFE）*/
    if (++heartbeat_cnt >= 200u) {
      heartbeat_cnt = 0u;
      /* 诊断：打印 FSM 状态、运行模式、故障位 */
      LOGINFO("FSM=%d mode=%d fault=%08X ctrl=%d",
              (int)StateMachine_GetState(&g_ds402_state_machine),
              (int)motor_data.state.State_Mode,
              (unsigned int)Safety_GetActiveFaultBits(),
              (int)motor_data.state.Control_Mode);
      /* 心跳帧携带诊断数据（替代 UART）:
       *   byte[0] = FSM state
       *   byte[1] = State_Mode  (0=IDLE,1=DET,2=RUN,3=GUARD)
       *   byte[2] = Control_Mode
       *   byte[3] = fault_bits[7:0]
       *   byte[4] = fault_bits[15:8]
       *   byte[5] = fault_bits[23:16]
       *   byte[6] = fault_bits[31:24]
       *   byte[7] = 0xAA (标识符) */
      uint32_t faults = Safety_GetActiveFaultBits();
      CAN_Frame hb = {
          .id  = (0x1Fu << 24) | (0x0001u << 8) | 0xFEu,  /* broadcast */
          .is_extended = true,
          .dlc = 8,
          .data = {
              (uint8_t)StateMachine_GetState(&g_ds402_state_machine),
              (uint8_t)motor_data.state.State_Mode,
              (uint8_t)motor_data.state.Control_Mode,
              (uint8_t)(faults),
              (uint8_t)(faults >> 8),
              (uint8_t)(faults >> 16),
              (uint8_t)(faults >> 24),
              0xAA,
          },
      };
      Protocol_SendFrame(&hb);
    }

    osDelay(5);
  }
}
