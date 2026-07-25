/********************************************************************************
 * @file        : bsp_usart.c
 * @author      : VectorFOC
 * @brief       : USARTdriver
 * @version     : V1.0
 * @date        : 2024 - 11 - 12
 *
 * @details:
 *  - USARTdriver,、、
 *
 * @note:
 *  -  DMA/IT/BLOCKING mode
 *
 * @history:
 *  V1.0:
 *    - ，
 *
 * Copyright (c) 2024 VectorFOC. All rights reserved.
 ********************************************************************************/
#include "bsp_usart.h"
#include "bsp_log.h"
#include "platform.h"
#include <string.h>
/* usart service instance, modules' info would be recoreded here using
 * USARTRegister() */
/* usart,usart */
static uint8_t idx;
static USARTInstance  usart_pool[DEVICE_USART_CNT];
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};
/**
 * @brief
 * start,,DMA,ITBLOCKING
 * @todo
 * ,DMA,ITBLOCKING
 *                 extern,module
 *
 * @param[in]      _instance:
 * @retval         note
 */
void USARTServiceInit(USARTInstance *_instance) {
  if (_instance == NULL || _instance->usart_handle == NULL ||
      _instance->recv_buff_size == 0U) {
    return;
  }
  /* Only set up DMA RX if a DMA handle is linked to the UART (e.g. VectorFOC).
   * On BOARD_XSTAR, USART2 has no DMA configured, so hdmarx is NULL. */
  if (_instance->usart_handle->hdmarx == NULL) {
    return;
  }
  HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff,
                               _instance->recv_buff_size);
  // dma half transferinterruptHAL_UARTEx_RxEventCallback()
  // HAL,DMAdone/doneIDLEinterruptHAL_UARTEx_RxEventCallback()
  // ,DMAinterrupt
  __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}
/**
 * @brief          USART
 * @param[in]      USART_config: USARTconfig
 * @retval         USART
 */
USARTInstance *USARTRegister(USART_Init_Config_s *USART_config) {
  if (USART_config == NULL || USART_config->usart_handle == NULL) {
    return NULL;
  }
  if (idx >= DEVICE_USART_CNT) //
  {
    LOGERROR("[bsp_usart] USART exceed max instance count!");
    return NULL;
  }
  for (uint8_t i = 0; i < idx; i++) // check
  {
    if (usart_instance[i]->usart_handle == USART_config->usart_handle) {
      return usart_instance[i];
    }
  }
  USARTInstance *usart = &usart_pool[idx];
  memset(usart, 0, sizeof(USARTInstance));
  usart->usart_handle = USART_config->usart_handle;
  usart->recv_buff_size =
      (USART_config->recv_buff_size == 0U ||
       USART_config->recv_buff_size > USART_RXBUFF_LIMIT)
          ? USART_RXBUFF_LIMIT
          : USART_config->recv_buff_size;
  usart->module_callback = USART_config->module_callback;
  usart_instance[idx++] = usart;
  USARTServiceInit(usart);
  return usart;
}
/**
 * @brief ：start/ DMA
 * @note protection，
 */
static void USART_StartTx(USARTInstance *_instance) {
  if (_instance == NULL || _instance->usart_handle == NULL) {
    return;
  }
  if (_instance->tx_head == _instance->tx_tail) {
    // buffer empty
    _instance->is_transmitting = 0;
    _instance->last_tx_len = 0;
    return;
  }
  // Determine contiguous length
  uint16_t head = _instance->tx_head;
  uint16_t tail = _instance->tx_tail;
  uint16_t len;
  if (head > tail) {
    len = head - tail;
  } else {
    len = USART_TXBUFF_SIZE - tail;
  }

  HAL_StatusTypeDef status = HAL_ERROR;
  _instance->is_transmitting = 1;
  _instance->last_tx_len = len; // Record length
  if (_instance->tx_mode == USART_TRANSFER_IT) {
    status = HAL_UART_Transmit_IT(_instance->usart_handle,
                                  &_instance->tx_buff[tail], len);
  } else if (_instance->tx_mode == USART_TRANSFER_DMA &&
             _instance->usart_handle->hdmatx != NULL) {
    status = HAL_UART_Transmit_DMA(_instance->usart_handle,
                                   &_instance->tx_buff[tail], len);
  }

  if (status != HAL_OK) {
    /* Leave queued bytes intact so a later send call can retry them. */
    _instance->is_transmitting = 0;
    _instance->last_tx_len = 0;
  }
}
/**
 * @brief          USART (mode)
 * @attention       DMA/IT
 * mode，，
 *
 * @param[in]      _instance:
 * @param[in]      send_buf:
 * @param[in]      send_size:
 * @param[in]      mode:      mode
 * @retval         USART
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,
               USART_TRANSFER_MODE mode) {
  if (_instance == NULL || _instance->usart_handle == NULL ||
      send_buf == NULL || send_size == 0U) {
    return;
  }
  if (mode == USART_TRANSFER_BLOCKING) {
    HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
    return;
  }
  if (mode != USART_TRANSFER_IT && mode != USART_TRANSFER_DMA) {
    return;
  }

  /* The producer can run in a task or an ISR while the completion callback
   * consumes the same ring. Publish the bytes, head, mode and kickoff as one
   * interrupt-atomic operation so the callback cannot observe a partial
   * enqueue. */
  CRITICAL_SECTION_BEGIN();
  if (_instance->is_transmitting == 0U || _instance->tx_mode == mode) {
    _instance->tx_mode = mode;
    for (uint16_t i = 0U; i < send_size; ++i) {
      uint16_t next_head =
          (uint16_t)((_instance->tx_head + 1U) % USART_TXBUFF_SIZE);
      if (next_head == _instance->tx_tail) {
        break;
      }
      _instance->tx_buff[_instance->tx_head] = send_buf[i];
      _instance->tx_head = next_head;
    }
    if (_instance->is_transmitting == 0U) {
      USART_StartTx(_instance);
    }
  }
  CRITICAL_SECTION_END();
}
/**
 * @brief          ,IT/DMA
 * @param[in]      _instance:
 * @retval         ready:1-,0-
 */
uint8_t USARTIsReady(USARTInstance *_instance) {
  return _instance != NULL && _instance->usart_handle != NULL &&
         _instance->usart_handle->gState == HAL_UART_STATE_READY;
}
/**
 * @brief
 * USARTinterrupt（dma/idleinterrupt，.uart）
 * @attention      __HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)dma half
 * transferinterruptHAL_UARTEx_RxEventCallback()
 *                 HAL,DMAdone/doneIDLEinterruptHAL_UARTEx_RxEventCallback()
 *                 ，DMAinterrupt
 *
 * @param[in]      huart: interrupt
 * @param[in]      Size: ,
 * @retval         note
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == NULL) {
    return;
  }
  for (uint8_t i = 0; i < idx;
       ++i) { // find the instance which is being handled
    if (huart ==
        usart_instance[i]
            ->usart_handle) { // call the callback function if it is not NULL
      uint16_t received =
          (Size < usart_instance[i]->recv_buff_size)
              ? Size
              : usart_instance[i]->recv_buff_size;
      if (usart_instance[i]->module_callback != NULL) {
        usart_instance[i]->module_callback();
      }
      memset(usart_instance[i]->recv_buff, 0, received); // buffer,
      if (usart_instance[i]->usart_handle->hdmarx != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle,
                                     usart_instance[i]->recv_buff,
                                     usart_instance[i]->recv_buff_size);
        __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx,
                            DMA_IT_HT);
      }
      return; // break the loop
    }
  }
}
/**
 * @brief
 * USARTerror（/error,,start）
 * @attention      error://error
 *
 * @param[in]      huart: error
 * @retval         note
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return;
  }
  for (uint8_t i = 0; i < idx; ++i) {
    if (huart == usart_instance[i]->usart_handle) {
      if (usart_instance[i]->usart_handle->hdmarx != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle,
                                     usart_instance[i]->recv_buff,
                                     usart_instance[i]->recv_buff_size);
        __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx,
                            DMA_IT_HT);
      }
      LOGWARNING(
          "[bsp_usart] USART error callback triggered, instance idx [%d]", i);
      return;
    }
  }
}
/**
 * @brief DMA done
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return;
  }
  for (uint8_t i = 0; i < idx; ++i) {
    if (huart == usart_instance[i]->usart_handle) {
      USARTInstance *ins = usart_instance[i];
      CRITICAL_SECTION_BEGIN();
      /* Ignore a spurious completion rather than advancing the consumer past
       * bytes which were never accepted by HAL. */
      if (ins->is_transmitting != 0U && ins->last_tx_len != 0U) {
        ins->tx_tail = (uint16_t)((ins->tx_tail + ins->last_tx_len) %
                                  USART_TXBUFF_SIZE);
        ins->is_transmitting = 0U;
        ins->last_tx_len = 0U;
      }
      USART_StartTx(ins);
      CRITICAL_SECTION_END();
      return;
    }
  }
}
