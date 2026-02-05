/**
 * @file app_init.c
 * @brief 应用层统一初始化实现
 * @note 原 robot.c 内容迁移至此
 */

#include "app_init.h"

#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_init.h"
#include "bsp_log.h"
#include "error_manager.h"
#include "error_types.h"
#include "hal_abstraction.h"
#include "led.h"
#include "manager.h"
#include "motor.h"
#include "motor/safety/safety_control.h"
#include "param_access.h"

void App_Init(void) {
  // 关闭中断,防止在初始化过程中发生中断
  __disable_irq();

  DWT_Delay(0.016f); // MT6816上电的16ms无输出数据
  BSPInit();         // 初始化DWT
  LogInit(&huart1);  // 初始化日志

  // 统一错误管理器初始化
  ErrorManager_Init();

  adc_bsp_init(); // 初始化ADC
  if (MHAL_PWM_Enable() != 0) {
    ERROR_REPORT(ERROR_HW_PWM_INIT, "PWM start failed");
    Error_Handler();
  }
  RGB_DisplayColorById(0);

  /* ===== System Initialization ===== */

  // 0. Safety & Detection System
  Detection_Init(NULL);
  Safety_Init(NULL);
  Safety_RegisterFaultCallback(Protocol_ReportFaultCallback);

  // 1. Parameter System
  Param_SystemInitOnce();

  // 2. Protocol Manager
  BSP_CAN_Init();
  Protocol_Init(PROTOCOL_INOVXIO);

  // 3. DS402 State Machine
  StateMachine_Init(&g_ds402_state_machine);

  // 4. Motor Initialization
  Init_Motor_No_Calib(&motor_data);

  __enable_irq();
}
