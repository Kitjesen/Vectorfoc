/**
 * @file can_transport.h
 * @brief CAN
 * @note  TransportInterface， CAN BSP
 */
#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H
#include "transport.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief get CAN
 * @return TransportInterface
 */
const TransportInterface *CAN_Transport_GetInterface(void);
/**
 * @brief init CAN
 * @note  BSP_CAN_Init()
 */
void CAN_Transport_Init(void);
#ifdef __cplusplus
}
#endif
#endif /* CAN_TRANSPORT_H */
