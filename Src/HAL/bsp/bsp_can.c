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
 * @brief CAN
 */
#include "bsp_can.h"
#include "board_config.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "protocol_types.h" // For CAN_Frame definition
#include "stdbool.h"
#include "string.h"
/* [FIX]  stdlib.h ( malloc) */
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "platform.h"
#define GET_CMD_TYPE(id) (((id) >> 24) & 0x1F)
#define GET_TARGET_ID(id) ((id) & 0xFF)
/* CAN（）
 * [FIX]  malloc， */
static FDCANInstance fdcan_instance_pool[FDCAN_MX_REGISTER_CNT];
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;
static volatile uint32_t s_tracked_tx_next_marker = 1U;
static volatile uint32_t s_tracked_tx_pending_marker = 0U;
static volatile uint32_t s_tracked_tx_pending_buffer_mask = 0U;
static volatile uint32_t s_tracked_tx_completed_marker = 0U;
/* ==========  ========== */
/**
 * @brief CANinit
 * @param _instance CAN
 * @return true，false
 */
static bool FDCANAddFilter(FDCANInstance *_instance) {
  HAL_StatusTypeDef result;
  FDCAN_FilterTypeDef fdcan_filter_conf;
  fdcan_filter_conf.IdType = FDCAN_EXTENDED_ID;  /* Protocol uses 29-bit IDs */
  fdcan_filter_conf.FilterIndex = 0;
  fdcan_filter_conf.FilterType = FDCAN_FILTER_RANGE;
  fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  fdcan_filter_conf.FilterID1 = 0x00000000;       /* Accept all extended IDs */
  fdcan_filter_conf.FilterID2 = 0x1FFFFFFF;
  result = HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &fdcan_filter_conf);
  return (result == HAL_OK);
}
/**
 * @brief CANinit（startFDCANinterrupt）
 * @return true，false
 */
static bool FDCANServiceInit(void) {
  HAL_StatusTypeDef result;
  result = HAL_FDCAN_Start(&HW_CAN);
  result |= HAL_FDCAN_ActivateNotification(&HW_CAN,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  result |= HAL_FDCAN_ActivateNotification(&HW_CAN,
                                           FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  result |= HAL_FDCAN_ActivateNotification(&HW_CAN,
                                           FDCAN_IT_TX_EVT_FIFO_NEW_DATA, 0);
  return result == HAL_OK;
}
/* ==========  ========== */

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

  return HAL_FDCAN_Init(&HW_CAN) == HAL_OK;
}

/**
 * @brief initCAN（）
 * @note App_Init，startFDCANinterrupt
 */
void BSP_CAN_Init(void) {
  if (!FDCANConfigureBaudrate(g_can_baudrate)) {
    LOGERROR("[bsp_can] Invalid or failed baudrate configuration, using 1Mbps");
    g_can_baudrate = BSP_CAN_BAUD_1M;
    if (!FDCANConfigureBaudrate(g_can_baudrate)) {
      LOGERROR("[bsp_can] CAN peripheral reinitialization failed");
      return;
    }
  }

  if (FDCANServiceInit()) {
    LOGINFO("[bsp_can] CAN Service Started (Unified Init)");
  } else {
    LOGERROR("[bsp_can] CAN Service Start Failed");
  }
}
/**
 * @brief FDCAN（）
 * @param config initconfig
 * @return CAN
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config) {
  if (!idx) {
    FDCANServiceInit(); // init
  }
  if (idx >= FDCAN_MX_REGISTER_CNT) {
    while (1)
      LOGERROR("[bsp_can] Max instances reached");
  }
  // checkID
  for (size_t i = 0; i < idx; i++) {
    if (fdcan_instance[i]->rx_id == config->rx_id &&
        fdcan_instance[i]->fdcan_handle == config->fdcan_handle) {
      while (1)
        LOGERROR("[bsp_can] ID collision");
    }
  }
  /* [FIX]  malloc */
  FDCANInstance *instance = &fdcan_instance_pool[idx];
  memset(instance, 0, sizeof(FDCANInstance));
  instance->txconf.Identifier = config->tx_id;
  instance->txconf.IdType = FDCAN_EXTENDED_ID;  /* Protocol uses 29-bit IDs */
  instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
  instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
  instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;
  instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;
  instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  instance->txconf.MessageMarker = 0;
  instance->fdcan_handle = config->fdcan_handle;
  instance->tx_id = config->tx_id;
  instance->rx_id = config->rx_id;
  instance->fdcan_module_callback = config->fdcan_module_callback;
  instance->id = config->id;
  FDCANAddFilter(instance);
  fdcan_instance[idx++] = instance;
  return instance;
}
/**
 * @brief FDCAN（）
 * @param _instance CAN
 * @return 1=, 0=
 * @note [FIX]
 */
uint8_t FDCANSendData(FDCANInstance *_instance) {
  if (_instance == NULL || _instance->fdcan_handle == NULL) {
    return 0;
  }
  return (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle,
                                        &_instance->txconf,
                                        _instance->tx_buff) == HAL_OK)
             ? 1
             : 0;
}
/**
 * @brief setCAN
 * @param _instance CAN
 * @param length （0-8）
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length) {
  switch (length) {
  case 0:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_0;
    break;
  case 1:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_1;
    break;
  case 2:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_2;
    break;
  case 3:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_3;
    break;
  case 4:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_4;
    break;
  case 5:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_5;
    break;
  case 6:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_6;
    break;
  case 7:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_7;
    break;
  case 8:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
    break;
  default:
    _instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
    break;
  }
}

static void BSP_CAN_FillTxHeader(const CAN_Frame *frame,
                                 FDCAN_TxHeaderTypeDef *header,
                                 uint32_t tx_event_control,
                                 uint32_t marker) {
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
/**
 * @brief CAN ()
 * @param frame CAN
 * @return true，false
 */
bool BSP_CAN_SendFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  FDCAN_TxHeaderTypeDef TxHeader;
  BSP_CAN_FillTxHeader(frame, &TxHeader, FDCAN_NO_TX_EVENTS, 0U);
  return (HAL_FDCAN_AddMessageToTxFifoQ(&HW_CAN, &TxHeader,
                                        (uint8_t *)frame->data) == HAL_OK);
}

bool BSP_CAN_SendTrackedFrame(const CAN_Frame *frame, BSP_CAN_TxTicket *ticket) {
  if (frame == NULL || ticket == NULL) {
    return false;
  }
  bool queued = false;
  memset(ticket, 0, sizeof(*ticket));
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
      (void)HAL_FDCAN_AbortTxRequest(&HW_CAN,
                                     s_tracked_tx_pending_buffer_mask);
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
  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) {
    FDCAN_RxHeaderTypeDef rxconf = {0};
    uint8_t can_rx_buff[8] = {0};
    // FIFOget
    if (HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) != HAL_OK) {
      continue;
    }
    //
    CAN_Frame frame = {0};
    frame.id = rxconf.Identifier;
    frame.dlc = rxconf.DataLength >> 16; // Approximation for Classic CAN
    frame.is_extended = (rxconf.IdType == FDCAN_EXTENDED_ID);
    frame.is_rtr = (rxconf.RxFrameType == FDCAN_REMOTE_FRAME);
    /* [FIX]  DLC  */
    uint8_t copy_len = (frame.dlc > 8) ? 8 : frame.dlc;
    memcpy(frame.data, can_rx_buff, copy_len);
    Protocol_QueueRxFrame(&frame);
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
