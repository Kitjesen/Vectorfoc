/**
 * @file    rng.c
 * @brief   Hardware random number generator configuration
 * @note    Clocked from HSI48
 * Copyright (c) 2025 STMicroelectronics. All rights reserved.
 */

#include "rng.h"

RNG_HandleTypeDef hrng;

void MX_RNG_Init(void) {
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_RNG_MspInit(RNG_HandleTypeDef *rngHandle) {
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (rngHandle->Instance == RNG) {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      Error_Handler();
    }
    __HAL_RCC_RNG_CLK_ENABLE();
  }
}

void HAL_RNG_MspDeInit(RNG_HandleTypeDef *rngHandle) {
  if (rngHandle->Instance == RNG) {
    __HAL_RCC_RNG_CLK_DISABLE();
  }
}
