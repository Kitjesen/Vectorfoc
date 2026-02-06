/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt service routines for STM32G4xx
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 */

#include "main.h"
#include "stm32g4xx_it.h"

/* External peripheral handles */
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_tim3_ch2;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim17;

/* ---- Cortex-M4 Exception Handlers ---- */

void NMI_Handler(void) {
  while (1) {
  }
}

void HardFault_Handler(void) {
  while (1) {
  }
}

void MemManage_Handler(void) {
  while (1) {
  }
}

void BusFault_Handler(void) {
  while (1) {
  }
}

void UsageFault_Handler(void) {
  while (1) {
  }
}

void DebugMon_Handler(void) {
}

/* ---- Peripheral Interrupt Handlers ---- */

/* DMA1 Ch1: TIM3_CH2 (WS2812 LED) */
void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_tim3_ch2);
}

/* DMA1 Ch2: ADC2 (temperature sensor) */
void DMA1_Channel2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc2);
}

/* DMA1 Ch3: USART1 RX */
void DMA1_Channel3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/* DMA1 Ch4: USART1 TX */
void DMA1_Channel4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/* ADC1 + ADC2 shared interrupt */
void ADC1_2_IRQHandler(void) {
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
}

/* USB low priority */
void USB_LP_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

/* FDCAN1 interrupt line 0 */
void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/* FDCAN1 interrupt line 1 */
void FDCAN1_IT1_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/* TIM1 trigger/commutation + TIM17 (HAL tick) shared interrupt */
void TIM1_TRG_COM_TIM17_IRQHandler(void) {
  if (htim1.Instance != NULL) {
    HAL_TIM_IRQHandler(&htim1);
  }
  if (htim17.Instance != NULL) {
    HAL_TIM_IRQHandler(&htim17);
  }
}

/* USART1 global interrupt */
void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart1);
}
