// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file bsp_can.h
 * @brief Portable classic-CAN BSP contract.
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
  BSP_CAN_BAUD_1M = 0U,
  BSP_CAN_BAUD_500K = 1U,
  BSP_CAN_BAUD_250K = 2U,
} BSP_CAN_BaudrateId;

typedef struct {
  uint32_t id;
  uint8_t data[8];
  uint8_t dlc;
  bool is_extended;
  bool is_rtr;
} BSP_CAN_Frame;

typedef struct {
  uint32_t marker; /**< Completion marker from the FDCAN Tx event FIFO. */
  uint32_t tx_buffer_mask; /**< HAL Tx FIFO/Q request buffer bit for abort. */
} BSP_CAN_TxTicket;
#define BSP_CAN_HAS_TRACKED_TX 1

/**
 * @brief Receive-frame callback invoked from the FDCAN ISR.
 * @note The callback must copy any data it needs before returning.
 */
typedef void (*BSP_CAN_RxCallback_t)(const BSP_CAN_Frame *frame);

/**
 * @brief Register the receive callback before starting the peripheral.
 * @return true when a non-NULL callback was installed.
 */
bool BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t cb);

/**
 * @brief Configure and start classic CAN with the selected baud rate.
 * @return true only when timing, filters, start, and notifications succeeded.
 */
bool BSP_CAN_Init(BSP_CAN_BaudrateId baudrate_id);
/** Return true when the hardware Tx FIFO can accept another frame. */
bool BSP_CAN_IsTxReady(void);
/**
 * @brief Queue one classic-CAN data frame.
 * @param frame Validated
 * standard or extended frame.
 * @return true if accepted by the hardware Tx
 * FIFO.
 */
bool BSP_CAN_SendFrame(const BSP_CAN_Frame *frame);
/**
 * @brief Send one CAN frame with Tx event tracking enabled.
 * @param frame  CAN frame.
 * @param ticket [out] Completion ticket, valid only on true.
 * @return true if queued with a unique marker; false on busy or HAL failure.
 * @note Only one tracked request may be pending. Normal BSP_CAN_SendFrame()
 *       does not create Tx events.
 */
bool BSP_CAN_SendTrackedFrame(const BSP_CAN_Frame *frame,
                              BSP_CAN_TxTicket *ticket);
/**
 * @brief Check and consume completion for a tracked send ticket.
 */
bool BSP_CAN_TxTicketIsComplete(const BSP_CAN_TxTicket *ticket);
/**
 * @brief Cancel a pending tracked send ticket after timeout/abort.
 */
void BSP_CAN_CancelTrackedSend(const BSP_CAN_TxTicket *ticket);
#ifdef __cplusplus
}
#endif
#endif
