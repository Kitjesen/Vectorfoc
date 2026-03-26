/**
 * @file transport.h
 * @brief
 * @note ，（CAN/USB/UART）
 *
 * :
 *   ┌─────────────────────┐
 *   │    Protocol Layer   │  (manager.c, inovxio, canopen, mit)
 *   ├─────────────────────┤
 *   │  TransportInterface │  ←
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
 * @brief
 */
typedef enum {
  TRANSPORT_CAN = 0,  /**< CAN  */
  TRANSPORT_USB = 1,  /**< USB-CDC */
  TRANSPORT_UART = 2, /**< UART */
} TransportType;
/**
 * @brief
 * @note
 */
typedef struct {
  TransportType type;   /**<  */
  uint32_t id;          /**< （CAN ID ）*/
  uint8_t data[64];     /**< （ CAN-FD）*/
  uint8_t len;          /**<  */
  bool is_extended;     /**< CAN  */
} TransportFrame;
/**
 * @brief
 * @param frame
 */
typedef void (*TransportRxCallback)(const TransportFrame *frame);
/**
 * @brief
 * @note （CAN、USB、UART）
 */
typedef struct {
  /**
   * @brief
   * @param frame
   * @return true=, false=
   */
  bool (*send)(const TransportFrame *frame);
  /**
   * @brief
   * @param cb
   * @return true=, false=
   */
  bool (*register_rx_callback)(TransportRxCallback cb);
  /**
   * @brief check
   * @return true=, false=
   */
  bool (*is_tx_ready)(void);
  /**
   * @brief
   */
  TransportType type;
} TransportInterface;
/* ============================================================================
 * CAN （ CAN_Frame  TransportFrame ）
 * ============================================================================ */
/**
 * @brief  CAN_Frame  TransportFrame
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
 * @brief  TransportFrame  CAN_Frame
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
