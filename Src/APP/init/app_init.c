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
#include "hal_encoder.h"
#include "hal_pwm.h"
#include "led.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "safety_control.h"

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

void App_Init(void) {
  __disable_irq();
#ifdef BOARD_XSTAR
  DWT_Delay(0.001f);
#else
  DWT_Delay(0.016f);
#endif
  BSPInit();
  LogInit(&HW_UART_DEBUG);
  ErrorManager_Init();
  adc_bsp_init();
  if (MHAL_PWM_Enable() != 0) {
    ERROR_REPORT(ERROR_HW_PWM_INIT, "PWM start failed");
    Error_Handler();
  }

  RGB_DisplayColorById(0);
  Detection_Init(NULL);
  Safety_Init(NULL);
  Safety_RegisterFaultCallback(App_ReportFaultCallback);

  Param_SystemInitOnce();
  BSP_CAN_Init();
  CAN_Transport_Init();
  Protocol_RegisterTransport(CAN_Transport_GetInterface());
  Protocol_Init(App_GetBootProtocol());
  StateMachine_Init(&g_ds402_state_machine);

#ifdef BOARD_XSTAR
  if (MHAL_Encoder_Init() != 0) {
    ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "X-STAR encoder init failed");
    Error_Handler();
  }
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  Hall_SetPolePairs((uint8_t)motor_data.parameters.pole_pairs);
#else
  Abz_SetPolePairs((uint8_t)motor_data.parameters.pole_pairs);
#endif
#endif

  Init_Motor_No_Calib(&motor_data);
  /* Open-loop from the start: prevents false encoder/voltage faults
   * before Hall signals stabilise and Vbus filter ramps up. */
  motor_data.state.Control_Mode = CONTROL_MODE_OPEN;
  __enable_irq();
}
