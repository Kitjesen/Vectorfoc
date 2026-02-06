/**
 * @file can_transport.h
 * @brief CAN 传输层适配器
 * @note 实现 TransportInterface，将 CAN BSP 适配为统一的传输接口
 */

#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#include "transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 CAN 传输接口
 * @return TransportInterface 指针
 */
const TransportInterface *CAN_Transport_GetInterface(void);

/**
 * @brief 初始化 CAN 传输适配器
 * @note 应在 BSP_CAN_Init() 之后调用
 */
void CAN_Transport_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TRANSPORT_H */
