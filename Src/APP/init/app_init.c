/**
 * @file app_init.c
 * @brief init
 * @note  robot.c
 */
#include "app_init.h"
#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_can.h"
#include "can_transport.h" // Transport layer adapter
#include "bsp_init.h"
#include "bsp_log.h"
#include "error_manager.h"
#include "error_types.h"
#include "hal_abstraction.h"
#include "hal_pwm.h" // For MHAL_PWM_Enable
#include "led.h"
#include "manager.h"
#include "motor.h"
#include "safety_control.h"
#include "param_access.h"

static bool App_ReportFaultCallback(uint32_t fault_bits, void *motor) {
  return Protocol_ReportFaultCallback(fault_bits, (MOTOR_DATA *)motor);
}

static ProtocolType App_GetBootProtocol(void) {
  if (g_protocol_type <= PROTOCOL_MIT) {
    return (ProtocolType)g_protocol_type;
  }
  g_protocol_type = PROTOCOL_INOVXIO;
  return PROTOCOL_INOVXIO;
}

void App_Init(void) {
  // interrupt,initinterrupt
  __disable_irq();
  DWT_Delay(0.016f); // MT681616msoutput
  BSPInit();         // initDWT
  LogInit(&HW_UART_DEBUG);  // Initialize debug log
  // errorinit
  ErrorManager_Init();
  adc_bsp_init(); // initADC
  if (MHAL_PWM_Enable() != 0) {
    ERROR_REPORT(ERROR_HW_PWM_INIT, "PWM start failed");
    Error_Handler();
  }
  RGB_DisplayColorById(0);
  /* ===== System Initialization ===== */
  // 0. Safety & Detection System
  Detection_Init(NULL);
  Safety_Init(NULL);
  Safety_RegisterFaultCallback(App_ReportFaultCallback);
  // 1. Parameter System
  Param_SystemInitOnce();
  // 2. Communication: CAN BSP + Transport + Protocol Manager
  BSP_CAN_Init();
  CAN_Transport_Init();
  Protocol_RegisterTransport(CAN_Transport_GetInterface());
  Protocol_Init(App_GetBootProtocol());
  // 3. DS402 State Machine
  StateMachine_Init(&g_ds402_state_machine);
  // 4. Motor Initialization
  Init_Motor_No_Calib(&motor_data);
  __enable_irq();
}
