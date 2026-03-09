/**
 * @file bsp_can.c
 * @brief CAN鎬荤嚎纭欢鎶借薄灞傚疄鐜?
 */

#include "bsp_can.h"
#include "board_config.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "main.h"
#include "protocol_types.h" // For CAN_Frame definition
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

#include "manager.h"
#include "motor.h"
#include "param_access.h"

#define GET_CMD_TYPE(id) (((id) >> 24) & 0x1F)
#define GET_TARGET_ID(id) ((id) & 0xFF)

/* CAN瀹炰緥绠＄悊锛堝吋瀹规棫浠ｇ爜锛?*/
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

/* ========== 鍐呴儴杈呭姪鍑芥暟 ========== */

/**
 * @brief CAN杩囨护鍣ㄥ垵濮嬪寲
 * @param _instance CAN瀹炰緥鎸囬拡
 * @return true鎴愬姛锛宖alse澶辫触
 */
static bool FDCANAddFilter(FDCANInstance *_instance) {
  HAL_StatusTypeDef result;
  FDCAN_FilterTypeDef fdcan_filter_conf;

  fdcan_filter_conf.IdType = FDCAN_STANDARD_ID;
  fdcan_filter_conf.FilterIndex = 0;
  fdcan_filter_conf.FilterType = FDCAN_FILTER_RANGE;
  fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  fdcan_filter_conf.FilterID1 = 0x0000;
  fdcan_filter_conf.FilterID2 = 0x0000;

  result = HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &fdcan_filter_conf);
  return (result == HAL_OK);
}

/**
 * @brief CAN鏈嶅姟鍒濆鍖栵紙鍚姩FDCAN骞舵縺娲讳腑鏂級
 * @return true鎴愬姛锛宖alse澶辫触
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

/* ========== 鍏叡鎺ュ彛鍑芥暟 ========== */

/**
 * @brief 鍒濆鍖朇AN鏈嶅姟锛堝澶栨帴鍙ｏ級
 * @note 鍦ˋpp_Init涓皟鐢紝鍚姩FDCAN鍜屼腑鏂?
 */
void BSP_CAN_Init(void) {
  if (FDCANServiceInit()) {
    LOGINFO("[bsp_can] CAN Service Started (Unified Init)");
  } else {
    LOGERROR("[bsp_can] CAN Service Start Failed");
  }
}

/**
 * @brief 娉ㄥ唽FDCAN瀹炰緥锛堝吋瀹规帴鍙ｏ級
 * @param config 鍒濆鍖栭厤缃?
 * @return CAN瀹炰緥鎸囬拡
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config) {
  if (!idx) {
    FDCANServiceInit(); // 寤惰繜鍒濆鍖?
  }
  if (idx >= FDCAN_MX_REGISTER_CNT) {
    while (1)
      LOGERROR("[bsp_can] Max instances reached");
  }

  // 妫€鏌D鍐茬獊
  for (size_t i = 0; i < idx; i++) {
    if (fdcan_instance[i]->rx_id == config->rx_id &&
        fdcan_instance[i]->fdcan_handle == config->fdcan_handle) {
      while (1)
        LOGERROR("[bsp_can] ID collision");
    }
  }

  FDCANInstance *instance = (FDCANInstance *)malloc(sizeof(FDCANInstance));
  memset(instance, 0, sizeof(FDCANInstance));

  instance->txconf.Identifier = config->tx_id;
  instance->txconf.IdType = FDCAN_STANDARD_ID;
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
 * @brief Send a queued frame through an FDCAN instance.
 * @param _instance Registered CAN instance.
 * @param timeout Timeout in ms; currently unused in the unified HAL path.
 * @return 1 on success, 0 on failure.
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout) {
  (void)timeout;

  if (_instance == NULL || _instance->fdcan_handle == NULL) {
    return 0U;
  }

  return (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle,
                                        &_instance->txconf,
                                        _instance->tx_buff) == HAL_OK)
             ? 1U
             : 0U;
}

/**
 * @brief Set CAN DLC.
 * @param _instance CAN instance.
 * @param length Payload length [0, 8].
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
 * @brief 鍙戦€丆AN甯?(鎶借薄鎺ュ彛)
 * @param frame CAN鍗忚甯?
 * @return true鎴愬姛锛宖alse澶辫触
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

/* ========== 鎺ユ敹涓柇鍥炶皟 ========== */

/**
 * @brief 缁熶竴鐨凜AN娑堟伅澶勭悊鍥炶皟
 * @param _hcan FDCAN鍙ユ焺
 * @param fifox FIFO缂栧彿锛團IFO0鎴朏IFO1锛?
 * @note 鐢盕IFO涓柇瑙﹀彂璋冪敤
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox) {
  static FDCAN_RxHeaderTypeDef rxconf;
  uint8_t can_rx_buff[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) {
    // 浠嶧IFO鑾峰彇娑堟伅
    HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);

    // 杞崲涓哄崗璁爤鏍煎紡骞跺鐞?
    CAN_Frame frame;
    frame.id = rxconf.Identifier;
    frame.dlc = rxconf.DataLength >> 16; // Approximation for Classic CAN
    frame.is_extended = (rxconf.IdType == FDCAN_EXTENDED_ID);
    frame.is_rtr = (rxconf.RxFrameType == FDCAN_REMOTE_FRAME);
    memcpy(frame.data, can_rx_buff, frame.dlc > 8 ? 8 : frame.dlc);

    Protocol_QueueRxFrame(&frame);
  }
}

/**
 * @brief FIFO0鏂版秷鎭腑鏂洖璋?
 * @param hfdcan FDCAN鍙ユ焺
 * @param RxFifo0ITs 涓柇鏍囧織
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0);
  }
}

/**
 * @brief FIFO1鏂版秷鎭腑鏂洖璋?
 * @param hfdcan FDCAN鍙ユ焺
 * @param RxFifo1ITs 涓柇鏍囧織
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
  }
}
