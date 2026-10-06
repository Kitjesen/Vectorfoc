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
 * @file application_init.c
 * @brief init
 * @note  robot.c
 */
#include "application.h"
#include "board_adc.h"
#include "board_can.h"
#include "current_calibration.h"
#include "board_cycle_counter.h"
#include "board_log.h"
#include "error_manager.h"
#include "error_types.h"
#include "hardware_interface.h"
#include "pwm_interface.h"
#include "status_led.h"
#include "protocol_dispatcher.h"
#include "motor_runtime.h"
#include "parameter_access.h"
#include "safety_manager.h"

static bool App_ReportFaultCallback(uint32_t fault_bits, void *motor) {
  return Protocol_ReportFaultCallback(fault_bits, (MOTOR_DATA *)motor);
}

static ProtocolType App_GetBootProtocol(void) {
  if (g_protocol_type <= PROTOCOL_MIT) {
    return (ProtocolType)g_protocol_type;
  }
  g_protocol_type = PROTOCOL_VECTOR;
  return PROTOCOL_VECTOR;
}

static bool App_CanEnterPoweredState(MotorState state) {
  if (state == STATE_CALIBRATING && motor_data.state.Sub_State == SUB_STATE_IDLE)
    return false;
  return Safety_CanEnable(&motor_data);
}

void App_Init(void) {
  __disable_irq();
  DWT_Delay(0.016f);
  DWT_Init(SystemCoreClock / 1000000UL); /* CPU cycles per microsecond. */
  LogInit(&HW_UART_DEBUG);
  ErrorManager_Init();
  RGB_DisplayColorById(0);
  Detection_Init(NULL);
  Safety_Init(NULL);
  Safety_RegisterFaultCallback(App_ReportFaultCallback);

  Param_SystemInitOnce();
  BSP_CAN_Init();
  Protocol_Init(App_GetBootProtocol());
  StateMachine_Init(&g_ds402_state_machine);
  StateMachine_SetPreCheckCallback(&g_ds402_state_machine, App_CanEnterPoweredState);

  Init_Motor_No_Calib(&motor_data);
  /* Every ISR consumer is initialized before arming ADCs and starting CH4.
   * No phase output is enabled here. Offset averaging runs one sample per ISR. */
  __disable_irq(); /* Earlier parameter/driver initialization may restore IRQs. */
  bool sampling_ok = adc_bsp_init() == 0;
  if (sampling_ok) sampling_ok = MHAL_PWM_StartSampling() == 0;
  CurrentCalib_BeginStartup(&motor_data, sampling_ok);
  if (!sampling_ok) {
    MHAL_PWM_Disable();
    ERROR_REPORT(ERROR_HW_ADC_INIT, "Sampling startup failed; motor remains disabled");
    StateMachine_EnterFault(&g_ds402_state_machine, FAULT_DRIVER_CHIP);
  }
  __enable_irq();
}
