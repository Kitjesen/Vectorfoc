/**
 * @file    stm32g4xx_hal_msp.c
 * @brief   Global MSP (MCU Support Package) initialization
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 */

#include "main.h"

/**
 * @brief  Global MSP init - called by HAL_Init().
 */
void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* PendSV at lowest priority for FreeRTOS context switching */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* Disable UCPD dead battery pull-down on CC pins */
  HAL_PWREx_DisableUCPDDeadBattery();
}
