/**
 * @file    tim.c
 * @brief   Timer peripheral configuration
 * @note    TIM1: 3-phase PWM (center-aligned, 20kHz) + ADC trigger on CH4
 *          TIM3: WS2812 LED data via DMA on CH2
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 */

#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch2;

/**
 * @brief  TIM1 init - center-aligned PWM for 3-phase motor drive.
 *         Period=4200 => 168MHz / (2*4200) = 20kHz
 *         CH1/CH2/CH3: motor phases, CH4: ADC trigger
 */
void MX_TIM1_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = MCPWM_RCR;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  /* Master: OC4REF triggers ADC injected conversion */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  /* PWM channels 1-3: motor phases (initial duty = 0) */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  /* CH4: ADC trigger pulse near counter peak (4190/4200) */
  sConfigOC.Pulse = 4190;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  /* Dead-time configuration */
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = MCPWM_DEADTIME_CLOCKS;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief  TIM3 init - PWM for WS2812 LED data line.
 *         Period=209 => 168MHz / 210 = 800kHz (WS2812 protocol)
 */
void MX_TIM3_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 209;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle) {
  if (tim_pwmHandle->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);

  } else if (tim_pwmHandle->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* DMA for WS2812 data stream */
    hdma_tim3_ch2.Instance = DMA1_Channel1;
    hdma_tim3_ch2.Init.Request = DMA_REQUEST_TIM3_CH2;
    hdma_tim3_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim3_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch2.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_tim3_ch2) != HAL_OK) {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_pwmHandle, hdma[TIM_DMA_ID_CC2], hdma_tim3_ch2);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (timHandle->Instance == TIM1) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*  PB13=CH1N, PB14=CH2N, PB15=CH3N (low-side)
     *  PA8=CH1, PA9=CH2, PA10=CH3 (high-side) */
    GPIO_InitStruct.Pin = LIN3_Pin | LIN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LIN1_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(LIN1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HIN3_Pin | HIN2_Pin | HIN1_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  } else if (timHandle->Instance == TIM3) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA4 = WS2812 data (TIM3_CH2) */
    GPIO_InitStruct.Pin = WS2812_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(WS2812_GPIO_Port, &GPIO_InitStruct);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle) {
  if (tim_pwmHandle->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);

  } else if (tim_pwmHandle->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_DMA_DeInit(tim_pwmHandle->hdma[TIM_DMA_ID_CC2]);
  }
}
