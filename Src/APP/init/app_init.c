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
 * @file app_init.c
 * @brief init
 * @note  robot.c
 */
#include "app_init.h"
#include "board_config.h"
#include "bsp_adc.h"
#ifdef BOARD_XSTAR
#include "hall_encoder.h"
#include "abz_encoder.h"
#include "xstar_bsp.h"
#endif
#include "bsp_can.h"
#include "can_transport.h"
#include "bsp_init.h"
#include "bsp_log.h"
#include "error_manager.h"
#include "error_types.h"
#include "hal_abstraction.h"
#include "hal_adc.h"
#include "hal_encoder.h"
#include "hal_pwm.h"
#include "led.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "safety_control.h"
#include "settings/runtime_settings.h"
#include <math.h>

static bool s_current_offsets_ready = false;
static bool s_initial_safety_scan_ready = false;
static volatile bool s_foc_runtime_ready = false;

bool App_IsFocRuntimeReady(void) { return s_foc_runtime_ready; }

static bool App_ReportFaultCallback(uint32_t fault_bits, void *motor) {
  return Protocol_ReportFaultCallback(fault_bits, (MOTOR_DATA *)motor);
}

static bool App_StatePreCheck(MotorState target_state) {
  if (target_state == STATE_OPERATION_ENABLED) {
    return s_current_offsets_ready && s_initial_safety_scan_ready &&
           !Safety_HasActiveFault();
  }

  if (target_state == STATE_CALIBRATING) {
    uint8_t fail_mask = 0u;
    uint8_t pass_mask = Motor_PreCalibCheck(&motor_data, &fail_mask);
    (void)fail_mask;
    return s_current_offsets_ready && pass_mask == 0x0Fu;
  }

  return true;
}

static ProtocolType App_GetBootProtocol(void) {
  if (g_protocol_type <= PROTOCOL_MIT) {
    return (ProtocolType)g_protocol_type;
  }
  g_protocol_type = PROTOCOL_VECTOR;
  return PROTOCOL_VECTOR;
}

static bool App_CaptureInitialFeedback(void) {
  const Motor_HAL_Handle_t *hal = motor_data.components.hal;
  if (hal == NULL || hal->adc == NULL || hal->adc->update == NULL ||
      hal->encoder == NULL || hal->encoder->get_data == NULL) {
    return false;
  }

  Motor_HAL_SensorData_t sensors = {0};
  hal->adc->update(&sensors);
  motor_data.algo_input.Ia = sensors.i_a;
  motor_data.algo_input.Ib = sensors.i_b;
  motor_data.algo_input.Ic = sensors.i_c;
  motor_data.algo_input.Vbus = sensors.v_bus;
  motor_data.feedback.temperature = sensors.temp;

  if (MHAL_Encoder_Update() != 0) {
    return false;
  }
  Motor_HAL_EncoderData_t encoder = {0};
  hal->encoder->get_data(&encoder);
  motor_data.feedback.position = encoder.position_rad / M_2PI;
  motor_data.feedback.velocity = encoder.velocity_rad / M_2PI;
  motor_data.feedback.phase_angle = encoder.elec_angle;
  motor_data.algo_input.theta_elec = encoder.elec_angle;

  return isfinite(sensors.i_a) && isfinite(sensors.i_b) &&
         isfinite(sensors.i_c) && isfinite(sensors.v_bus) &&
         isfinite(sensors.temp) && isfinite(encoder.position_rad) &&
         isfinite(encoder.velocity_rad) && isfinite(encoder.elec_angle);
}

void App_Init(void) {
  __disable_irq();
  s_foc_runtime_ready = false;
  BSPInit();
#ifdef BOARD_XSTAR
  DWT_Delay(0.001f);
#else
  DWT_Delay(0.016f);
#endif
  HAL_WatchdogFeed();

  LogInit(&HW_UART_DEBUG);
  ErrorManager_Init();

  RGB_DisplayColorById(0);
  StateMachine_Init(&g_ds402_state_machine);
  StateMachine_SetPreCheckCallback(&g_ds402_state_machine, App_StatePreCheck);
  Detection_Init(NULL);
  Safety_Init(NULL);
  __enable_irq();

  Param_SystemInitOnce();
  if (!adc_bsp_init()) {
    ERROR_REPORT(ERROR_HW_ADC_INIT, "ADC startup failed");
    Error_Handler();
  }
  if (MHAL_PWM_StartSampling() != 0) {
    ERROR_REPORT(ERROR_HW_PWM_INIT, "PWM sampling trigger start failed");
    Error_Handler();
  }
  HAL_WatchdogFeed();
  if (MHAL_ADC_CalibrateCurrent() != 0) {
    ERROR_REPORT(ERROR_HW_ADC_INIT, "Current offset calibration failed");
    Error_Handler();
  }
  s_current_offsets_ready = true;
  MHAL_PWM_Disable();

  if (MHAL_Encoder_Init() != 0) {
    ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "Encoder init failed");
    Error_Handler();
  }
  if (RuntimeSettings_ApplyEncoderOffset() != 0) {
    ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "Encoder offset restore failed");
  }
  if (!App_CaptureInitialFeedback()) {
    Safety_TriggerFault(FAULT_POSITION_INIT, &motor_data,
                        &g_ds402_state_machine);
  } else {
    uint32_t initial_faults = Detection_Check_Slow(&motor_data);
    if (initial_faults != FAULT_NONE) {
      Safety_TriggerFault(initial_faults, &motor_data,
                          &g_ds402_state_machine);
    }
  }
  s_initial_safety_scan_ready = true;

  BSP_CAN_Init();
  CAN_Transport_Init();
  Protocol_RegisterTransport(CAN_Transport_GetInterface());
  Protocol_Init(App_GetBootProtocol());
  Safety_RegisterFaultCallback(App_ReportFaultCallback);

  Init_Motor_No_Calib(&motor_data);
  MHAL_PWM_Disable();
  /*
   * Parameter storage is intentionally initialized before the encoder,
   * protocol and motor runtime exist.  Install the APP adapter only now, then
   * replay the persisted values once every target module is ready.
   */
  RuntimeSettings_InstallAdapter();
  Param_ApplyRuntimeState();
  /* Publish readiness only after every object the 20 kHz ADC ISR dereferences
   * has been initialized.  A data memory barrier prevents a future Cortex-M
   * target from observing the flag before the preceding state writes. */
  __DMB();
  s_foc_runtime_ready = true;
  HAL_WatchdogFeed();
}
