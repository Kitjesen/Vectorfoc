/**
 * @file    dma.c
 * @brief   DMA controller initialization
 * @note    DMA1 Ch1-Ch4 used by TIM3/ADC2/USART1
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 */

#include "dma.h"

void MX_DMA_Init(void) {
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA1_Ch1: TIM3_CH2 (WS2812) */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  /* DMA1_Ch2: ADC2 (temperature) */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  /* DMA1_Ch3: USART1_RX */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  /* DMA1_Ch4: USART1_TX */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}
