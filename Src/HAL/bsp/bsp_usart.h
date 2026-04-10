/********************************************************************************
 * @file        : bsp_usart.h
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
#ifndef BSP_USART_H
#define BSP_USART_H
#include "main.h"
#include "usart.h"
#include <stdint.h>
#define DEVICE_USART_CNT 5     //
#define USART_RXBUFF_LIMIT 256 // buff,
/**
 * @brief ,
 */
typedef void (*usart_module_callback)();
/**
 * @brief mode
 */
#pragma pack(1)
typedef enum {
  USART_TRANSFER_NONE = 0,
  USART_TRANSFER_BLOCKING,
  USART_TRANSFER_IT,
  USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;
/**
 * @brief
 */
#define USART_TXBUFF_SIZE 2048 //
typedef struct {
  uint8_t recv_buff[USART_RXBUFF_LIMIT]; // buff
  uint8_t recv_buff_size;                //
  UART_HandleTypeDef *usart_handle;      // usart_handle
  usart_module_callback module_callback; //
  /*  */
  uint8_t tx_buff[USART_TXBUFF_SIZE];
  volatile uint16_t tx_head;
  volatile uint16_t tx_tail;
  volatile uint8_t is_transmitting; // 1: DMA
  volatile uint16_t last_tx_len;    // DMA
} USARTInstance;
/**
 * @brief initconfig
 */
typedef struct {
  uint8_t recv_buff_size;                //
  UART_HandleTypeDef *usart_handle;      // usart_handle
  usart_module_callback module_callback; //
} USART_Init_Config_s;
#pragma pack()
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
void USARTServiceInit(USARTInstance *_instance);
/**
 * @brief          USART
 * @param[in]      USART_config: USARTconfig
 * @retval         USART
 */
USARTInstance *USARTRegister(USART_Init_Config_s *USART_config);
/**
 * @brief          USART
 * @attention
 * ,IT/DMAdone.
 *                 ,DMA/IT,USARTIsReady(),module.
 * @todo           USARTInstance?
 *
 * @param[in]      _instance:
 * @param[in]      send_buf:
 * @param[in]      send_size:
 * @param[in]      mode:      mode
 * @retval         USART
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,
               USART_TRANSFER_MODE mode);
/**
 * @brief          ,IT/DMA
 * @param[in]      _instance:
 * @retval         ready:1-,0-
 */
uint8_t USARTIsReady(USARTInstance *_instance);
#endif // BSP_USART_H
