// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "board_can.h"
#include "board_configuration.h"
#include "board_log.h"
#include "main.h"
#include "protocol_dispatcher.h"
#include <stdbool.h>
#include <string.h>

static bool FDCANServiceInit(void) {
  HAL_StatusTypeDef result = HAL_FDCAN_Start(&HW_CAN);
  result |= HAL_FDCAN_ActivateNotification(
      &HW_CAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  result |= HAL_FDCAN_ActivateNotification(
      &HW_CAN, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  return result == HAL_OK;
}

void BSP_CAN_Init(void) {
  if (FDCANServiceInit()) {
    LOGINFO("[bsp_can] CAN Service Started (Unified Init)");
  } else {
    LOGERROR("[bsp_can] CAN Service Start Failed");
  }
}

static uint32_t FDCAN_DataLength(uint8_t dlc) {
  switch (dlc) {
  case 0: return FDCAN_DLC_BYTES_0;
  case 1: return FDCAN_DLC_BYTES_1;
  case 2: return FDCAN_DLC_BYTES_2;
  case 3: return FDCAN_DLC_BYTES_3;
  case 4: return FDCAN_DLC_BYTES_4;
  case 5: return FDCAN_DLC_BYTES_5;
  case 6: return FDCAN_DLC_BYTES_6;
  case 7: return FDCAN_DLC_BYTES_7;
  case 8: return FDCAN_DLC_BYTES_8;
  default: return FDCAN_DLC_BYTES_8;
  }
}

bool BSP_CAN_SendFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = frame->id;
  header.IdType = frame->is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  header.TxFrameType = frame->is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DataLength(frame->dlc);
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  return HAL_FDCAN_AddMessageToTxFifoQ(&HW_CAN, &header,
                                       (uint8_t *)frame->data) == HAL_OK;
}

static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo) {
  static FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo)) {
    HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data);
    CAN_Frame frame = {0};
    frame.id = rx_header.Identifier;
    frame.dlc = (uint8_t)(rx_header.DataLength >> 16);
    frame.is_extended = rx_header.IdType == FDCAN_EXTENDED_ID;
    frame.is_rtr = rx_header.RxFrameType == FDCAN_REMOTE_FRAME;
    uint8_t copy_len = frame.dlc > 8 ? 8 : frame.dlc;
    memcpy(frame.data, rx_data, copy_len);
    Protocol_QueueRxFrame(&frame);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t interrupts) {
  if ((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0);
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t interrupts) {
  if ((interrupts & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
  }
}
