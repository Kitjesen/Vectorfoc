/**
 * @file bsp_can.c
 * @brief CAN总线硬件抽象层实现
 */

#include "bsp_can.h"
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

/* CAN实例管理（兼容旧代码） */
static FDCANInstance *fdcan_instance[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;

/* ========== 内部辅助函数 ========== */

/**
 * @brief CAN过滤器初始化
 * @param _instance CAN实例指针
 * @return true成功，false失败
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
 * @brief CAN服务初始化（启动FDCAN并激活中断）
 * @return true成功，false失败
 */
static bool FDCANServiceInit(void) {
  HAL_StatusTypeDef result;
  result = HAL_FDCAN_Start(&hfdcan1);
  result |= HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  result |= HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  return result == HAL_OK;
}

/* ========== 公共接口函数 ========== */

/**
 * @brief 初始化CAN服务（对外接口）
 * @note 在RobotInit中调用，启动FDCAN和中断
 */
void BSP_CAN_Init(void) {
  if (FDCANServiceInit()) {
    LOGINFO("[bsp_can] CAN Service Started (Unified Init)");
  } else {
    LOGERROR("[bsp_can] CAN Service Start Failed");
  }
}

/**
 * @brief 注册FDCAN实例（兼容接口）
 * @param config 初始化配置
 * @return CAN实例指针
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config) {
  if (!idx) {
    FDCANServiceInit(); // 延迟初始化
  }
  if (idx >= FDCAN_MX_REGISTER_CNT) {
    while (1)
      LOGERROR("[bsp_can] Max instances reached");
  }

  // 检查ID冲突
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
 * @brief 通过FDCAN发送数据
 * @param _instance CAN实例
 * @param timeout 超时时间（ms）
  return 1;
}

/**
 * @brief 设置CAN数据长度
 * @param _instance CAN实例
 * @param length 数据长度（0-8字节）
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
 * @brief 发送CAN帧 (抽象接口)
 * @param frame CAN协议帧
 * @return true成功，false失败
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

  // Use hfdcan1 directly for now (Single instance simplification)
  return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader,
                                        (uint8_t *)frame->data) == HAL_OK);
}

/* ========== 接收中断回调 ========== */

/**
 * @brief 统一的CAN消息处理回调
 * @param _hcan FDCAN句柄
 * @param fifox FIFO编号（FIFO0或FIFO1）
 * @note 由FIFO中断触发调用
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox) {
  static FDCAN_RxHeaderTypeDef rxconf;
  uint8_t can_rx_buff[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) {
    // 从FIFO获取消息
    HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff);

    // 转换为协议栈格式并处理
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
 * @brief FIFO0新消息中断回调
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0);
  }
}

/**
 * @brief FIFO1新消息中断回调
 * @param hfdcan FDCAN句柄
 * @param RxFifo1ITs 中断标志
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
    FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
  }
}
