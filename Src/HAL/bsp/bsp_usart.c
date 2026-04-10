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
  if (idx >= DEVICE_USART_CNT) //
  {
    while (1)
      LOGERROR("[bsp_usart] USART exceed max instance count!");
  }
  for (uint8_t i = 0; i < idx; i++) // check
  {
    if (usart_instance[i]->usart_handle == USART_config->usart_handle) {
      while (1)
        LOGERROR("[bsp_usart] USART instance already registered!");
    }
  }
  USARTInstance *usart = &usart_pool[idx];
  memset(usart, 0, sizeof(USARTInstance));
  usart->usart_handle = USART_config->usart_handle;
  usart->recv_buff_size = USART_config->recv_buff_size;
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
  if (_instance->tx_head == _instance->tx_tail) {
    // buffer empty
    _instance->is_transmitting = 0;
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
  _instance->is_transmitting = 1;
  _instance->last_tx_len = len; // Record length
  HAL_UART_Transmit_DMA(_instance->usart_handle, &_instance->tx_buff[tail],
                        len);
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
  if (mode == USART_TRANSFER_BLOCKING) {
    HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
    return;
  }
  // DMA/IT Mode: Push to Ring Buffer
  // Simple push logic (can be optimized with memcpy)
  for (uint16_t i = 0; i < send_size; i++) {
    uint16_t next_head = (_instance->tx_head + 1) % USART_TXBUFF_SIZE;
    if (next_head != _instance->tx_tail) {
      _instance->tx_buff[_instance->tx_head] = send_buf[i];
      _instance->tx_head = next_head;
    } else {
      // Buffer overflow
      break;
    }
  }
  // If IDLE, trigger transmission
  if (_instance->is_transmitting == 0) {
    USART_StartTx(_instance);
  }
}
/**
 * @brief          ,IT/DMA
 * @param[in]      _instance:
 * @retval         ready:1-,0-
 */
uint8_t USARTIsReady(USARTInstance *_instance) {
  if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
    return 0;
  else
    return 1;
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
  for (uint8_t i = 0; i < idx;
       ++i) { // find the instance which is being handled
    if (huart ==
        usart_instance[i]
            ->usart_handle) { // call the callback function if it is not NULL
      if (usart_instance[i]->module_callback != NULL) {
        usart_instance[i]->module_callback();
        memset(usart_instance[i]->recv_buff, 0,
               Size); // buffer,
      }
      HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle,
                                   usart_instance[i]->recv_buff,
                                   usart_instance[i]->recv_buff_size);
      __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
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
  for (uint8_t i = 0; i < idx; ++i) {
    if (huart == usart_instance[i]->usart_handle) {
      HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle,
                                   usart_instance[i]->recv_buff,
                                   usart_instance[i]->recv_buff_size);
      __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
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
  for (uint8_t i = 0; i < idx; ++i) {
    if (huart == usart_instance[i]->usart_handle) {
      USARTInstance *ins = usart_instance[i];
      // Advance tail based on what was transmitted
      uint16_t head = ins->tx_head;
      uint16_t tail = ins->tx_tail;
      uint16_t len;
      if (head > tail) {
        len = head - tail;
      } else {
        len = USART_TXBUFF_SIZE - tail;
      }
      ins->tx_tail = (ins->tx_tail + len) % USART_TXBUFF_SIZE;
      // Trigger next batch
      USART_StartTx(ins);
      return;
    }
  }
}
