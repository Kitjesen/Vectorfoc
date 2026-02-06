/**
 * @file transport.h
 * @brief 传输层抽象接口
 * @note 定义通用的传输接口，使协议层独立于具体的物理传输（CAN/USB/UART）
 *
 * 架构:
 *   ┌─────────────────────┐
 *   │    Protocol Layer   │  (manager.c, inovxio, canopen, mit)
 *   ├─────────────────────┤
 *   │  TransportInterface │  ← 本文件定义的抽象接口
 *   ├─────────────────────┤
 *   │   Transport Impl    │  (CAN BSP, USB-CDC, UART)
 *   └─────────────────────┘
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include "protocol_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 传输类型枚举
 */
typedef enum {
  TRANSPORT_CAN = 0,  /**< CAN 总线 */
  TRANSPORT_USB = 1,  /**< USB-CDC */
  TRANSPORT_UART = 2, /**< UART */
} TransportType;

/**
 * @brief 通用传输帧结构
 * @note 统一不同传输方式的数据格式
 */
typedef struct {
  TransportType type;   /**< 传输类型 */
  uint32_t id;          /**< 帧标识（CAN ID 或命令标识）*/
  uint8_t data[64];     /**< 数据载荷（兼容 CAN-FD）*/
  uint8_t len;          /**< 数据长度 */
  bool is_extended;     /**< CAN 扩展帧标志 */
} TransportFrame;

/**
 * @brief 接收回调函数类型
 * @param frame 接收到的帧
 */
typedef void (*TransportRxCallback)(const TransportFrame *frame);

/**
 * @brief 传输层接口结构
 * @note 不同的传输实现（CAN、USB、UART）需要实现这些函数
 */
typedef struct {
  /**
   * @brief 发送数据帧
   * @param frame 要发送的帧
   * @return true=发送成功, false=发送失败
   */
  bool (*send)(const TransportFrame *frame);

  /**
   * @brief 注册接收回调
   * @param cb 接收回调函数
   * @return true=注册成功, false=注册失败
   */
  bool (*register_rx_callback)(TransportRxCallback cb);

  /**
   * @brief 检查发送缓冲区是否可用
   * @return true=可发送, false=缓冲区满
   */
  bool (*is_tx_ready)(void);

  /**
   * @brief 传输类型
   */
  TransportType type;

} TransportInterface;

/* ============================================================================
 * CAN 传输适配器（从 CAN_Frame 到 TransportFrame 的转换）
 * ============================================================================ */

/**
 * @brief 将 CAN_Frame 转换为 TransportFrame
 */
static inline void Transport_FromCANFrame(const CAN_Frame *can,
                                          TransportFrame *tf) {
  tf->type = TRANSPORT_CAN;
  tf->id = can->id;
  tf->len = can->dlc;
  tf->is_extended = can->is_extended;
  for (uint8_t i = 0; i < can->dlc && i < 8; i++) {
    tf->data[i] = can->data[i];
  }
}

/**
 * @brief 将 TransportFrame 转换为 CAN_Frame
 */
static inline void Transport_ToCANFrame(const TransportFrame *tf,
                                        CAN_Frame *can) {
  can->id = tf->id;
  can->dlc = (tf->len > 8) ? 8 : tf->len;
  can->is_extended = tf->is_extended;
  can->is_rtr = false;
  for (uint8_t i = 0; i < can->dlc; i++) {
    can->data[i] = tf->data[i];
  }
}

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_H */
