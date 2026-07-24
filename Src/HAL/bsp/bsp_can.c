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
 * @file bsp_can.c
 * @brief Portable classic-CAN BSP implemented with STM32
 * FDCAN.
 */
#include "bsp_can.h"
#include "board_config.h"
#include "bsp_log.h"
#include "main.h"
#include "platform.h"
#include "string.h"
static volatile uint32_t s_tracked_tx_next_marker = 1U;
static volatile uint32_t s_tracked_tx_pending_marker = 0U;
static volatile uint32_t s_tracked_tx_pending_buffer_mask = 0U;
static volatile uint32_t s_tracked_tx_completed_marker = 0U;
static BSP_CAN_RxCallback_t s_rx_callback;

bool BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t callback) {
  s_rx_callback = callback;
  return callback != NULL;
}
/** Configure standard and extended data-frame filters. */
static bool FDCANConfigureRxFilters(void) {
  FDCAN_FilterTypeDef filter = {0};
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x00000000U;

  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterID2 = 0x1FFFFFFFU;
  if (HAL_FDCAN_ConfigFilter(&HW_CAN, &filter) != HAL_OK) {
    return false;
  }

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(&HW_CAN, &filter) != HAL_OK) {
    return false;
  }

  return HAL_FDCAN_ConfigGlobalFilter(&HW_CAN, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE) == HAL_OK;
}
/** Start FDCAN and enable receive/Tx-event notifications. */
static bool FDCANServiceInit(void) {
  if (HAL_FDCAN_Start(&HW_CAN) != HAL_OK) {
    return false;
  }

  const uint32_t notifications = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                 FDCAN_IT_TX_EVT_FIFO_NEW_DATA;
  if (HAL_FDCAN_ActivateNotification(&HW_CAN, notifications, 0U) != HAL_OK) {
    (void)HAL_FDCAN_Stop(&HW_CAN);
    return false;
  }
  return true;
}
static bool FDCANConfigureBaudrate(uint8_t baudrate_id) {
  uint32_t rate_divisor;
  switch (baudrate_id) {
  case BSP_CAN_BAUD_1M:
    rate_divisor = 1U;
    break;
  case BSP_CAN_BAUD_500K:
    rate_divisor = 2U;
    break;
  case BSP_CAN_BAUD_250K:
    rate_divisor = 4U;
    break;
  default:
    return false;
  }

  if (HAL_FDCAN_DeInit(&HW_CAN) != HAL_OK) {
    return false;
  }

#if defined(BOARD_XSTAR)
  /* X-STAR: 170 MHz kernel clock, 17 time quanta: 1 + 13 + 3. */
  uint32_t prescaler = 10U * rate_divisor;
  HW_CAN.Init.NominalPrescaler = prescaler;
  HW_CAN.Init.NominalSyncJumpWidth = 3U;
  HW_CAN.Init.NominalTimeSeg1 = 13U;
  HW_CAN.Init.NominalTimeSeg2 = 3U;
  HW_CAN.Init.DataPrescaler = prescaler;
  HW_CAN.Init.DataSyncJumpWidth = 3U;
  HW_CAN.Init.DataTimeSeg1 = 13U;
  HW_CAN.Init.DataTimeSeg2 = 3U;
#else
  /* 168 MHz kernel clock, 28 time quanta: 1 + 20 + 7. */
  uint32_t prescaler = 6U * rate_divisor;
  HW_CAN.Init.NominalPrescaler = prescaler;
  HW_CAN.Init.NominalSyncJumpWidth = 7U;
  HW_CAN.Init.NominalTimeSeg1 = 20U;
  HW_CAN.Init.NominalTimeSeg2 = 7U;
  HW_CAN.Init.DataPrescaler = prescaler;
  HW_CAN.Init.DataSyncJumpWidth = 7U;
  HW_CAN.Init.DataTimeSeg1 = 20U;
  HW_CAN.Init.DataTimeSeg2 = 7U;
#endif

  /* One range filter per identifier format. Keep these counts local to the
   * BSP re-init so generated board init order cannot silently disable a
   * protocol family. */
  HW_CAN.Init.StdFiltersNbr = 1U;
  HW_CAN.Init.ExtFiltersNbr = 1U;
  return HAL_FDCAN_Init(&HW_CAN) == HAL_OK;
}

/** Configure and start the selected classic-CAN baud rate. */
bool BSP_CAN_Init(BSP_CAN_BaudrateId baudrate_id) {
  if (s_rx_callback == NULL) {
    LOGERROR("[bsp_can] RX callback must be registered before CAN start");
    return false;
  }
  if (!FDCANConfigureBaudrate((uint8_t)baudrate_id)) {
    LOGERROR("[bsp_can] Invalid or failed baudrate configuration");
    return false;
  }
  if (!FDCANConfigureRxFilters()) {
    LOGERROR("[bsp_can] CAN receive filter configuration failed");
    return false;
  }
  s_tracked_tx_pending_marker = 0U;
  s_tracked_tx_pending_buffer_mask = 0U;
  s_tracked_tx_completed_marker = 0U;
  if (!FDCANServiceInit()) {
    LOGERROR("[bsp_can] CAN Service Start Failed");
    return false;
  }
  LOGINFO("[bsp_can] CAN Service Started");
  return true;
}
bool BSP_CAN_IsTxReady(void) {
  return HAL_FDCAN_GetTxFifoFreeLevel(&HW_CAN) > 0U;
}

static bool BSP_CAN_FrameIsValid(const BSP_CAN_Frame *frame) {
  if (frame == NULL || frame->dlc > sizeof(frame->data) || frame->is_rtr) {
    return false;
  }
  const uint32_t max_id = frame->is_extended ? 0x1FFFFFFFU : 0x7FFU;
  return frame->id <= max_id;
}
static void BSP_CAN_FillTxHeader(const BSP_CAN_Frame *frame,
                                 FDCAN_TxHeaderTypeDef *header,
                                 uint32_t tx_event_control, uint32_t marker) {
  memset(header, 0, sizeof(*header));
  header->Identifier = frame->id;
  header->IdType = frame->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  header->TxFrameType = frame->is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  switch (frame->dlc) {
  case 0:
    header->DataLength = FDCAN_DLC_BYTES_0;
    break;
  case 1:
    header->DataLength = FDCAN_DLC_BYTES_1;
    break;
  case 2:
    header->DataLength = FDCAN_DLC_BYTES_2;
    break;
  case 3:
    header->DataLength = FDCAN_DLC_BYTES_3;
    break;
  case 4:
    header->DataLength = FDCAN_DLC_BYTES_4;
    break;
  case 5:
    header->DataLength = FDCAN_DLC_BYTES_5;
    break;
  case 6:
    header->DataLength = FDCAN_DLC_BYTES_6;
    break;
  case 7:
    header->DataLength = FDCAN_DLC_BYTES_7;
    break;
  case 8:
    header->DataLength = FDCAN_DLC_BYTES_8;
    break;
  default:
    header->DataLength = FDCAN_DLC_BYTES_8;
    break;
  }
  header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header->BitRateSwitch = FDCAN_BRS_OFF;
  header->FDFormat = FDCAN_CLASSIC_CAN;
  header->TxEventFifoControl = tx_event_control;
  header->MessageMarker = marker;
}
/** Queue one validated classic-CAN data frame for transmission. */
bool BSP_CAN_SendFrame(const BSP_CAN_Frame *frame) {
  if (!BSP_CAN_FrameIsValid(frame)) {
    return false;
  }
  FDCAN_TxHeaderTypeDef TxHeader;
  BSP_CAN_FillTxHeader(frame, &TxHeader, FDCAN_NO_TX_EVENTS, 0U);
  return (HAL_FDCAN_AddMessageToTxFifoQ(&HW_CAN, &TxHeader,
                                        (uint8_t *)frame->data) == HAL_OK);
}

bool BSP_CAN_SendTrackedFrame(const BSP_CAN_Frame *frame,
                              BSP_CAN_TxTicket *ticket) {
  if (ticket == NULL) {
    return false;
  }
  memset(ticket, 0, sizeof(*ticket));
  if (!BSP_CAN_FrameIsValid(frame)) {
    return false;
  }
  bool queued = false;
  CRITICAL_SECTION_BEGIN();
  if (s_tracked_tx_pending_marker == 0U &&
      s_tracked_tx_completed_marker == 0U) {
    uint32_t marker = s_tracked_tx_next_marker & 0xFFU;
    if (marker == 0U) {
      marker = 1U;
    }
    s_tracked_tx_next_marker = (marker == 0xFFU) ? 1U : (marker + 1U);

    FDCAN_TxHeaderTypeDef tx_header;
    BSP_CAN_FillTxHeader(frame, &tx_header, FDCAN_STORE_TX_EVENTS, marker);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&HW_CAN, &tx_header,
                                      (uint8_t *)frame->data) == HAL_OK) {
      uint32_t buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&HW_CAN);
      s_tracked_tx_pending_marker = marker;
      s_tracked_tx_pending_buffer_mask = buffer_mask;
      ticket->marker = marker;
      ticket->tx_buffer_mask = buffer_mask;
      queued = true;
    }
  }
  CRITICAL_SECTION_END();
  return queued;
}

bool BSP_CAN_TxTicketIsComplete(const BSP_CAN_TxTicket *ticket) {
  if (ticket == NULL || ticket->marker == 0U) {
    return false;
  }
  bool complete = false;
  CRITICAL_SECTION_BEGIN();
  if (s_tracked_tx_completed_marker == ticket->marker) {
    s_tracked_tx_completed_marker = 0U;
    complete = true;
  }
  CRITICAL_SECTION_END();
  return complete;
}

void BSP_CAN_CancelTrackedSend(const BSP_CAN_TxTicket *ticket) {
  if (ticket == NULL || ticket->marker == 0U) {
    return;
  }
  CRITICAL_SECTION_BEGIN();
  if (s_tracked_tx_pending_marker == ticket->marker) {
    if (s_tracked_tx_pending_buffer_mask != 0U &&
        s_tracked_tx_pending_buffer_mask == ticket->tx_buffer_mask) {
      (void)HAL_FDCAN_AbortTxRequest(&HW_CAN, s_tracked_tx_pending_buffer_mask);
    }
    s_tracked_tx_pending_marker = 0U;
    s_tracked_tx_pending_buffer_mask = 0U;
  }
  if (s_tracked_tx_completed_marker == ticket->marker) {
    s_tracked_tx_completed_marker = 0U;
  }
  CRITICAL_SECTION_END();
}
/* ========== interrupt ========== */
/**
 * @brief CAN
 * @param _hcan FDCAN
 * @param fifox FIFO（FIFO0FIFO1）
 * @note FIFOinterrupt
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox) {
  if (_hcan != &HW_CAN) {
    return;
  }
  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox) != 0U) {
    FDCAN_RxHeaderTypeDef rxconf = {0};
    uint8_t can_rx_buff[64] = {0};
    if (HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) != HAL_OK) {
      break;
    }

    BSP_CAN_Frame frame = {0};
    frame.id = rxconf.Identifier;
    frame.dlc = (uint8_t)(rxconf.DataLength >> 16);
    frame.is_extended = (rxconf.IdType == FDCAN_EXTENDED_ID);
    frame.is_rtr = (rxconf.RxFrameType == FDCAN_REMOTE_FRAME);
    if (!BSP_CAN_FrameIsValid(&frame)) {
      continue;
    }
    memcpy(frame.data, can_rx_buff, frame.dlc);
    if (s_rx_callback != NULL) {
      s_rx_callback(&frame);
    }
  }
}
/**
 * @brief FIFO0interrupt
 * @param hfdcan FDCAN
 * @param RxFifo0ITs interrupt
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0);
  }
}
/**
 * @brief FIFO1interrupt
 * @param hfdcan FDCAN
 * @param RxFifo1ITs interrupt
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
  }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t TxEventFifoITs) {
  if (hfdcan != &HW_CAN) {
    return;
  }
  if ((TxEventFifoITs & FDCAN_IT_TX_EVT_FIFO_NEW_DATA) == RESET) {
    return;
  }

  FDCAN_TxEventFifoTypeDef tx_event;
  while (HAL_FDCAN_GetTxEvent(hfdcan, &tx_event) == HAL_OK) {
    uint32_t marker = tx_event.MessageMarker & 0xFFU;
    if (marker != 0U && marker == s_tracked_tx_pending_marker) {
      s_tracked_tx_pending_marker = 0U;
      s_tracked_tx_pending_buffer_mask = 0U;
      s_tracked_tx_completed_marker = marker;
    }
  }
}
