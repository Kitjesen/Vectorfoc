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
/* 接收回调（由 can_transport.c 通过 BSP_CAN_SetRxCallback 注册）
 * 原来直接调用 manager.h 中的 Protocol_QueueRxFrame，
 * 改为回调模式消除 HAL→COMM 反向依赖。 */
static BSP_CAN_RxCallback_t s_rx_cb = NULL;

void BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t cb)
{
    s_rx_cb = cb;
}
#define GET_CMD_TYPE(id) (((id) >> 24) & 0x1F)
#define GET_TARGET_ID(id) ((id) & 0xFF)
/* CAN（）
 * [FIX]  malloc， */
static FDCANInstance fdcan_instance_pool[FDCAN_MX_REGISTER_CNT];
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;
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
  return result == HAL_OK;
}
/* ==========  ========== */
/**
 * @brief initCAN（）
 * @note App_Init，startFDCANinterrupt
 */
void BSP_CAN_Init(void) {
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
  TxHeader.Identifier = frame->id;
  TxHeader.IdType = frame->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = frame->is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  // Mapping DLC
  switch (frame->dlc) {
  case 0:
    TxHeader.DataLength = FDCAN_DLC_BYTES_0;
    break;
  case 1:
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    break;
  case 2:
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    break;
  case 3:
    TxHeader.DataLength = FDCAN_DLC_BYTES_3;
    break;
  case 4:
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    break;
  case 5:
    TxHeader.DataLength = FDCAN_DLC_BYTES_5;
    break;
  case 6:
    TxHeader.DataLength = FDCAN_DLC_BYTES_6;
    break;
  case 7:
    TxHeader.DataLength = FDCAN_DLC_BYTES_7;
    break;
  case 8:
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    break;
  default:
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    break;
  }
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  return (HAL_FDCAN_AddMessageToTxFifoQ(&HW_CAN, &TxHeader,
                                        (uint8_t *)frame->data) == HAL_OK);
}
/* ========== interrupt ========== */
/**
 * @brief CAN
 * @param _hcan FDCAN
 * @param fifox FIFO（FIFO0FIFO1）
 * @note FIFOinterrupt
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox) {
  static FDCAN_RxHeaderTypeDef rxconf;
  uint8_t can_rx_buff[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) {
    // FIFOget
    HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);
    //
    CAN_Frame frame;
    frame.id = rxconf.Identifier;
    frame.dlc = rxconf.DataLength >> 16; // Approximation for Classic CAN
    frame.is_extended = (rxconf.IdType == FDCAN_EXTENDED_ID);
    frame.is_rtr = (rxconf.RxFrameType == FDCAN_REMOTE_FRAME);
    /* [FIX]  DLC  */
    uint8_t copy_len = (frame.dlc > 8) ? 8 : frame.dlc;
    memcpy(frame.data, can_rx_buff, copy_len);
    if (s_rx_cb != NULL) {
      s_rx_cb(&frame);
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
