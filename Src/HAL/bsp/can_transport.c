/**
 * @file can_transport.c
 * @brief CAN 传输层适配器实现
 * @note 将 BSP_CAN 封装为 TransportInterface
 */

#include "can_transport.h"
#include "bsp_can.h"
#include "hal_abstraction.h" // For HAL_CAN_IsTxMailboxAvailable

/**
 * @brief 通过 CAN 发送 TransportFrame
 */
static bool CAN_Transport_Send(const TransportFrame *frame) {
  if (frame == NULL) {
    return false;
  }

  // 转换为 CAN_Frame
  CAN_Frame can_frame;
  Transport_ToCANFrame(frame, &can_frame);

  // 调用底层 BSP 发送
  return BSP_CAN_SendFrame(&can_frame);
}

/**
 * @brief 注册接收回调
 * @note CAN 接收由 BSP 中断处理，回调通过 Protocol_QueueRxFrame 实现
 *       此函数目前仅作为接口占位
 */
static bool CAN_Transport_RegisterRxCallback(TransportRxCallback cb) {
  (void)cb;
  // CAN 接收已通过 bsp_can.c 中的 FDCANFIFOxCallback 实现
  // 该回调调用 Protocol_QueueRxFrame()，无需额外注册
  return true;
}

/**
 * @brief 检查发送缓冲区是否就绪
 */
static bool CAN_Transport_IsTxReady(void) {
  // 使用 HAL 抽象层检查
  return HAL_CAN_IsTxMailboxAvailable();
}

/**
 * @brief CAN 传输接口实例
 */
static const TransportInterface s_can_transport = {
    .send = CAN_Transport_Send,
    .register_rx_callback = CAN_Transport_RegisterRxCallback,
    .is_tx_ready = CAN_Transport_IsTxReady,
    .type = TRANSPORT_CAN,
};

/**
 * @brief 获取 CAN 传输接口
 */
const TransportInterface *CAN_Transport_GetInterface(void) {
  return &s_can_transport;
}

/**
 * @brief 初始化 CAN 传输适配器
 */
void CAN_Transport_Init(void) {
  // 目前无需额外初始化，BSP_CAN_Init() 已完成硬件初始化
}
