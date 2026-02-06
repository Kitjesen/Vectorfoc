/**
 * @file    stm32g4xx_hal_timebase_tim.c
 * @brief   HAL time base using TIM17 (1ms tick)
 * @note    Called automatically by HAL_Init() and HAL_RCC_ClockConfig()
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"

TIM_HandleTypeDef htim17;

/**
 * @brief  Configure TIM17 as 1ms HAL tick source.
 * @param  TickPriority  Tick interrupt priority
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  RCC_ClkInitTypeDef clkconfig;
  uint32_t uwTimclock;
  uint32_t uwPrescalerValue;
  uint32_t pFLatency;
  HAL_StatusTypeDef status;

  __HAL_RCC_TIM17_CLK_ENABLE();

  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  uwTimclock = HAL_RCC_GetPCLK2Freq();

  /* Prescaler for 1MHz counter clock */
  uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

  htim17.Instance = TIM17;
  htim17.Init.Period = (1000000U / 1000U) - 1U;  /* 1ms period */
  htim17.Init.Prescaler = uwPrescalerValue;
  htim17.Init.ClockDivision = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;

  status = HAL_TIM_Base_Init(&htim17);
  if (status == HAL_OK) {
    status = HAL_TIM_Base_Start_IT(&htim17);
    if (status == HAL_OK) {
      HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
      if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
        HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      } else {
        status = HAL_ERROR;
      }
    }
  }

  return status;
}

void HAL_SuspendTick(void) {
  __HAL_TIM_DISABLE_IT(&htim17, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void) {
  __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
}
