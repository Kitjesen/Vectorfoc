// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ENCODER_SPI_TRANSFER_H
#define ENCODER_SPI_TRANSFER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef ENCODER_SPI_TIMEOUT_US
#define ENCODER_SPI_TIMEOUT_US 20U
#endif

/*
 * The encoder is sampled from the 20 kHz control ISR. HAL's millisecond
 * timeout can therefore block hundreds of control periods on a bus fault.
 * This transfer mirrors the HAL 8-bit full-duplex polling path, but uses the
 * already-enabled DWT cycle counter to enforce a microsecond-scale deadline.
 */
static inline HAL_StatusTypeDef
EncoderSPI_TransmitReceiveBounded(SPI_HandleTypeDef *hspi, const uint8_t *tx,
                                  uint8_t *rx, uint16_t size) {
  if (hspi == NULL || tx == NULL || rx == NULL || size == 0U ||
      hspi->Init.DataSize != SPI_DATASIZE_8BIT ||
      hspi->Init.Direction != SPI_DIRECTION_2LINES) {
    return HAL_ERROR;
  }
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
    return HAL_ERROR;
  }
  if (hspi->Lock == HAL_LOCKED || hspi->State != HAL_SPI_STATE_READY) {
    return HAL_BUSY;
  }

  hspi->Lock = HAL_LOCKED;
  hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  uint32_t previous_fifo_threshold =
      READ_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
  SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
  if ((hspi->Instance->CR1 & SPI_CR1_SPE) == 0U) {
    __HAL_SPI_ENABLE(hspi);
  }

  uint16_t tx_remaining = size;
  uint16_t rx_remaining = size;
  uint16_t tx_index = 0U;
  uint16_t rx_index = 0U;
  bool tx_allowed = true;
  uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  uint32_t timeout_cycles =
      (cycles_per_us > 0U ? cycles_per_us : 1U) * ENCODER_SPI_TIMEOUT_US;
  uint32_t start_cycles = DWT->CYCCNT;
  HAL_StatusTypeDef result = HAL_OK;

  while (tx_remaining > 0U || rx_remaining > 0U) {
    if (tx_allowed && tx_remaining > 0U &&
        __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) {
      *(__IO uint8_t *)&hspi->Instance->DR = tx[tx_index++];
      tx_remaining--;
      tx_allowed = false;
    }
    if (rx_remaining > 0U && __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
      rx[rx_index++] = *(__IO uint8_t *)&hspi->Instance->DR;
      rx_remaining--;
      tx_allowed = true;
    }
    if ((uint32_t)(DWT->CYCCNT - start_cycles) >= timeout_cycles) {
      result = HAL_TIMEOUT;
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
      break;
    }
  }

  while (result == HAL_OK && __HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {
    if ((uint32_t)(DWT->CYCCNT - start_cycles) >= timeout_cycles) {
      result = HAL_TIMEOUT;
      hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;
    }
  }

  if (previous_fifo_threshold == 0U) {
    CLEAR_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
  }
  hspi->State = HAL_SPI_STATE_READY;
  hspi->Lock = HAL_UNLOCKED;
  return result;
}

#endif /* ENCODER_SPI_TRANSFER_H */
