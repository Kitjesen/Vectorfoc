/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt service routines for STM32G4xx
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 */

#include "main.h"
#include "stm32g4xx_it.h"

/* External peripheral handles */
extern ADC_HandleTypeDef   hadc1;
extern ADC_HandleTypeDef   hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef   htim1;

#ifdef BOARD_XSTAR
extern TIM_HandleTypeDef   htim3;
extern TIM_HandleTypeDef   htim17;
extern UART_HandleTypeDef  huart2;
#else
extern PCD_HandleTypeDef   hpcd_USB_FS;
extern DMA_HandleTypeDef   hdma_adc2;
extern DMA_HandleTypeDef   hdma_tim3_ch2;
extern DMA_HandleTypeDef   hdma_usart1_rx;
extern DMA_HandleTypeDef   hdma_usart1_tx;
extern UART_HandleTypeDef  huart1;
extern TIM_HandleTypeDef   htim17;
#endif

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

/* ADC1 + ADC2 shared interrupt（FOC ISR，两块板通用） */
void ADC1_2_IRQHandler(void) {
  HAL_ADC_IRQHandler(&hadc1);
}

/* FDCAN1 interrupt line 0 */
void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/* FDCAN1 interrupt line 1 */
void FDCAN1_IT1_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

#ifdef BOARD_XSTAR
/* ---- X-STAR-S 专用中断 ---- */

/* TIM17: HAL tick 源（与 TIM1_TRG_COM 共享中断线） */
void TIM1_TRG_COM_TIM17_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim17);
}

/* TIM3: Hall 传感器跳变检测（CC1 中断） */
void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim3);
}

/* USART2: 调试串口 */
void USART2_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart2);
}

/* TIM1 更新中断（HAL Tick 用 TIM17，此处仅处理 TIM1 本身事件） */
void TIM1_UP_TIM16_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim1);
}

#else
/* ---- VectorFOC 专用中断 ---- */

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

/* USB low priority */
void USB_LP_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
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

#endif /* BOARD_XSTAR */
