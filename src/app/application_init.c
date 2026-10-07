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
#include "communication_task.h"
#include "encoder_interface.h"
#include "parameter_bindings.h"
#include "encoder_calibration_settings.h"
#include "runtime_settings.h"
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

static volatile bool s_foc_runtime_ready;
bool App_IsFocRuntimeReady(void) { return s_foc_runtime_ready; }

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
  if (state == STATE_CALIBRATING) {
    uint8_t fail_mask = 0u;
    if (motor_data.state.Sub_State == SUB_STATE_IDLE ||
        Motor_PreCalibCheck(&motor_data, &fail_mask) != 0x0Fu)
      return false;
  }
  return Safety_CanEnable(&motor_data);
}

void App_Init(void) {
  __disable_irq();
  s_foc_runtime_ready = false;
  DWT_Init(SystemCoreClock / 1000000UL);
  DWT_Delay(0.016f);
  HAL_WatchdogFeed();
  LogInit(&HW_UART_DEBUG);
  ErrorManager_Init();
  RGB_DisplayColorById(0);
  Detection_Init(NULL);
  Safety_Init(NULL);
  Safety_RegisterFaultCallback(App_ReportFaultCallback);
  StateMachine_Init(&g_ds402_state_machine);
  StateMachine_SetPreCheckCallback(&g_ds402_state_machine, App_CanEnterPoweredState);
  MHAL_PWM_Disable();
  /* Peripheral waits need the millisecond tick. The ISR readiness gate protects
   * every consumer until initialization and passive sampling are ready. */
  __enable_irq();
  if (ParameterBindingsSettings_Install() != PARAM_OK) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE, "Parameter binding initialization failed");
    Error_Handler();
    return;
  }
  EncoderCalibrationSettings_InstallAdapter();
  if (MHAL_Encoder_Init() != 0) {
    ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "Encoder initialization failed");
    Error_Handler();
    return;
  }
  Param_SystemInitOnce();
  Protocol_Init(App_GetBootProtocol());
  CommTask_Init();
  if (!BSP_CAN_Init((BSP_CAN_BaudrateId)g_can_baudrate)) {
    ERROR_REPORT(ERROR_HW_CAN_INIT, "CAN initialization failed");
    Error_Handler();
    return;
  }
  Init_Motor_No_Calib(&motor_data);
  RuntimeSettings_InstallAdapter();
  Param_ApplyRuntimeState();
  if (RuntimeSettings_ApplyEncoderOffset() != 0)
    ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "Encoder offset restore failed");

  __disable_irq();
  bool sampling_ok = adc_bsp_init() == 0;
  if (sampling_ok) sampling_ok = MHAL_PWM_StartSampling() == 0;
  CurrentCalib_BeginStartup(&motor_data, sampling_ok);
  if (!sampling_ok) {
    MHAL_PWM_Disable();
    ERROR_REPORT(ERROR_HW_ADC_INIT, "Sampling startup failed; motor remains disabled");
    StateMachine_EnterFault(&g_ds402_state_machine, FAULT_DRIVER_CHIP);
  }
#if !defined(TEST_ENV)
  __DMB();
#endif
  s_foc_runtime_ready = sampling_ok;
  HAL_WatchdogFeed();
  __enable_irq();
}
