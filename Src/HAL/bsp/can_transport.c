/**
 * @file can_transport.c
 * @brief CAN
 * @note  BSP_CAN  TransportInterface
 */
#include "can_transport.h"
#include "bsp_can.h"
#include "hal_abstraction.h" // For HAL_CAN_IsTxMailboxAvailable
/**
 * @brief  CAN  TransportFrame
 */
static bool CAN_Transport_Send(const TransportFrame *frame) {
  if (frame == NULL) {
    return false;
  }
  //  CAN_Frame
  CAN_Frame can_frame;
  Transport_ToCANFrame(frame, &can_frame);
  //  BSP
  return BSP_CAN_SendFrame(&can_frame);
}
/**
 * @brief
 * @note CAN  BSP interrupt， Protocol_QueueRxFrame
 *
 */
static bool CAN_Transport_RegisterRxCallback(TransportRxCallback cb) {
  (void)cb;
  // CAN  bsp_can.c  FDCANFIFOxCallback
  //  Protocol_QueueRxFrame()，
  return true;
}
/**
 * @brief checkready
 */
static bool CAN_Transport_IsTxReady(void) {
  //  HAL check
  return HAL_CAN_IsTxMailboxAvailable();
}
/**
 * @brief CAN
 */
static const TransportInterface s_can_transport = {
    .send = CAN_Transport_Send,
    .register_rx_callback = CAN_Transport_RegisterRxCallback,
    .is_tx_ready = CAN_Transport_IsTxReady,
    .type = TRANSPORT_CAN,
};
/**
 * @brief get CAN
 */
const TransportInterface *CAN_Transport_GetInterface(void) {
  return &s_can_transport;
}
/**
 * @brief init CAN
 */
void CAN_Transport_Init(void) {
  // init，BSP_CAN_Init() doneinit
}
