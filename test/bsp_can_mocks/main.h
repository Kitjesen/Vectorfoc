// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef TEST_BSP_CAN_MAIN_H
#define TEST_BSP_CAN_MAIN_H

#include <stdint.h>

#define RESET 0U

typedef enum {
  HAL_OK = 0,
  HAL_ERROR = 1,
  HAL_BUSY = 2,
  HAL_TIMEOUT = 3,
} HAL_StatusTypeDef;

typedef struct {
  uint32_t NominalPrescaler;
  uint32_t NominalSyncJumpWidth;
  uint32_t NominalTimeSeg1;
  uint32_t NominalTimeSeg2;
  uint32_t DataPrescaler;
  uint32_t DataSyncJumpWidth;
  uint32_t DataTimeSeg1;
  uint32_t DataTimeSeg2;
  uint32_t StdFiltersNbr;
  uint32_t ExtFiltersNbr;
} FDCAN_InitTypeDef;

typedef struct {
  void *Instance;
  FDCAN_InitTypeDef Init;
} FDCAN_HandleTypeDef;

typedef struct {
  uint32_t IdType;
  uint32_t FilterIndex;
  uint32_t FilterType;
  uint32_t FilterConfig;
  uint32_t FilterID1;
  uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct {
  uint32_t Identifier;
  uint32_t IdType;
  uint32_t TxFrameType;
  uint32_t DataLength;
  uint32_t ErrorStateIndicator;
  uint32_t BitRateSwitch;
  uint32_t FDFormat;
  uint32_t TxEventFifoControl;
  uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
  uint32_t Identifier;
  uint32_t IdType;
  uint32_t RxFrameType;
  uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

typedef struct {
  uint32_t MessageMarker;
} FDCAN_TxEventFifoTypeDef;

#define FDCAN_STANDARD_ID 0x11U
#define FDCAN_EXTENDED_ID 0x22U
#define FDCAN_DATA_FRAME 0x33U
#define FDCAN_REMOTE_FRAME 0x44U
#define FDCAN_FILTER_RANGE 0x55U
#define FDCAN_FILTER_TO_RXFIFO0 0x66U
#define FDCAN_REJECT 0x77U
#define FDCAN_REJECT_REMOTE 0x88U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_RX_FIFO1 1U

#define FDCAN_DLC_BYTES_0 (0U << 16)
#define FDCAN_DLC_BYTES_1 (1U << 16)
#define FDCAN_DLC_BYTES_2 (2U << 16)
#define FDCAN_DLC_BYTES_3 (3U << 16)
#define FDCAN_DLC_BYTES_4 (4U << 16)
#define FDCAN_DLC_BYTES_5 (5U << 16)
#define FDCAN_DLC_BYTES_6 (6U << 16)
#define FDCAN_DLC_BYTES_7 (7U << 16)
#define FDCAN_DLC_BYTES_8 (8U << 16)

#define FDCAN_ESI_ACTIVE 0x101U
#define FDCAN_BRS_OFF 0x102U
#define FDCAN_CLASSIC_CAN 0x103U
#define FDCAN_NO_TX_EVENTS 0x104U
#define FDCAN_STORE_TX_EVENTS 0x105U

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1U << 0)
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE (1U << 1)
#define FDCAN_IT_TX_EVT_FIFO_NEW_DATA (1U << 2)

HAL_StatusTypeDef HAL_FDCAN_DeInit(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t notifications,
                                                 uint32_t buffers);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *header,
                                                uint8_t *data);
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *hfdcan);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *hfdcan,
                                           uint32_t buffer_mask);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t fifo);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *header,
                                         uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *hfdcan,
                                       FDCAN_TxEventFifoTypeDef *event);

#endif
