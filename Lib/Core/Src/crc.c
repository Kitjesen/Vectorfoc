/**
 * @file    crc.c
 * @brief   Hardware CRC peripheral configuration
 * Copyright (c) 2025 STMicroelectronics. All rights reserved.
 */

#include "crc.h"

CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void) {
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_CRC_MspInit(CRC_HandleTypeDef *crcHandle) {
  if (crcHandle->Instance == CRC) {
    __HAL_RCC_CRC_CLK_ENABLE();
  }
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef *crcHandle) {
  if (crcHandle->Instance == CRC) {
    __HAL_RCC_CRC_CLK_DISABLE();
  }
}
