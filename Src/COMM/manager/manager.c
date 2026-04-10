/**
 * @file manager.c
 * @brief  -
 *
 * :
 *   1. : Inovxio/CANopen/MIT
 *   2. :
 *   3. : motor
 *
 * :
 *   CAN → Protocol_ProcessRxFrame() →  → motor
 *                                       ↓
 *   CAN ← Protocol_BuildFeedback() ← state
 *
 * :
 *   Protocol_Init(PROTOCOL_INOVXIO);        // init
 *   Protocol_ProcessRxFrame(&can_frame);    // CANinterrupt
 *   Protocol_SendFrame(&can_frame);         // CAN
    CAN 。
 */
#include "main.h" // For FDCAN support
#include "manager.h"
#include "transport.h"
#include "bsp_can.h" //  CAN
#include "bsp_dwt.h"
#include "protocol/canopen/canopen_protocol.h"
#include "error_manager.h"
#include "error_types.h"
#include "executor/executor.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"
#include "protocol/inovxio/inovxio_protocol.h"
#include "protocol/mit/mit_protocol.h"
#include "motor.h"
#include "safety_control.h" // For Safety_GetActiveFaultBits
#include "param_access.h"
#include "param_table.h"
#include "device_id.h"
#include <string.h>
#define CAN_BROADCAST_ADDR 0x7F   ///< CAN
#define INOVXIO_CMD_GET_ID 0x00   ///< InovxiogetID
#define PROTOCOL_RX_QUEUE_LEN 32U ///< Rx ring buffer length
/*  */
static ProtocolType s_current_protocol = PROTOCOL_INOVXIO;
/* （ NULL， BSP_CAN ）*/
static const TransportInterface *s_transport = NULL;
/* ISR->Task Rx ring buffer */
static CAN_Frame s_rx_queue[PROTOCOL_RX_QUEUE_LEN];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;
static volatile uint32_t s_rx_dropped = 0;
static volatile bool s_rx_overflow = false;
/* Communication statistics */
static CommStats_t s_comm_stats = {0};
static volatile uint32_t s_rx_queue_peak = 0;
static bool Protocol_DequeueRxFrame(CAN_Frame *out) {
  if (out == NULL) {
    return false;
  }
  bool has_frame = false;
  __disable_irq();
  if (s_rx_head != s_rx_tail) {
    *out = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % PROTOCOL_RX_QUEUE_LEN);
    has_frame = true;
  }
  __enable_irq();
  return has_frame;
}
/**
 * @brief
 */
void Protocol_RegisterTransport(const TransportInterface *transport) {
  s_transport = transport;
}
/**
 * @brief init
 */
void Protocol_Init(ProtocolType default_protocol) {
  s_current_protocol = default_protocol;
  // init
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    ProtocolPrivate_Init();
    break;
  case PROTOCOL_CANOPEN:
    ProtocolCANopen_Init();
    break;
  case PROTOCOL_MIT:
    ProtocolMIT_Init();
    break;
  default:
    // Inovxio
    s_current_protocol = PROTOCOL_INOVXIO;
    ProtocolPrivate_Init();
    break;
  }
}
/**
 * @brief set
 */
void Protocol_SetType(ProtocolType protocol) {
  if (protocol != s_current_protocol) {
    s_current_protocol = protocol;
    Protocol_Init(protocol);
  }
}
/**
 * @brief get
 */
ProtocolType Protocol_GetType(void) { return s_current_protocol; }
/**
 * @brief CAN ()
 */
ParseResult Protocol_ParseFrame(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    ERROR_REPORT(ERROR_COMM_INVALID_FRAME, "Invalid CAN frame");
    return PARSE_ERR_INVALID_FRAME;
  }
  ParseResult result;
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    result = ProtocolPrivate_Parse(frame, cmd);
    break;
  case PROTOCOL_CANOPEN:
    result = ProtocolCANopen_Parse(frame, cmd);
    break;
  case PROTOCOL_MIT:
    result = ProtocolMIT_Parse(frame, cmd);
    break;
  default:
    result = PARSE_ERR_INVALID_FRAME;
    break;
  }
  // error（OKerror，UNKNOWN_ID）
  if (result != PARSE_OK && result != PARSE_UNKNOWN_ID) {
    s_comm_stats.parse_errors++;
    ERROR_REPORT(ERROR_COMM_PARSE_FAILED, "CAN frame parse error");
  }
  return result;
}
/**
 * @brief feedback
 */
bool Protocol_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildFeedback(status, frame);
  case PROTOCOL_CANOPEN:
    return ProtocolCANopen_BuildFeedback(status, frame);
  case PROTOCOL_MIT:
    return ProtocolMIT_BuildFeedback(status, frame);
  default:
    return false;
  }
}
/**
 * @brief fault
 */
bool Protocol_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildFault(fault_code, 0, frame);
  case PROTOCOL_CANOPEN:
    return ProtocolCANopen_BuildFault(fault_code, frame);
  case PROTOCOL_MIT:
    return ProtocolMIT_BuildFault(fault_code, frame);
  default:
    return false;
  }
}
/**
 * @brief param
 */
bool Protocol_BuildParamResponse(uint16_t param_index, float value,
                                 CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildParamResponse(param_index, value, frame);
    // param，
    // case PROTOCOL_CANOPEN: ...
  default:
    return false;
  }
}
/**
 * @brief  Build calibration status frame (CMD 0x09)
 */
bool Protocol_BuildCalibStatus(const MotorStatus *status, CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildCalibStatus(status, frame);
  default:
    return false;
  }
}
/**
 * @brief CAN
 * @note ， BSP_CAN_SendFrame
 */
bool Protocol_SendFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  bool result = false;
  /*  */
  if (s_transport != NULL && s_transport->send != NULL) {
    TransportFrame tf;
    Transport_FromCANFrame(frame, &tf);
    result = s_transport->send(&tf);
  } else {
    /*  BSP CAN （）*/
    result = BSP_CAN_SendFrame(frame);
  }
  if (result) {
    s_comm_stats.tx_frames_total++;
  } else {
    s_comm_stats.tx_frames_failed++;
  }
  return result;
}
/**
 * @brief Rx (ISRsafety)
 */
bool Protocol_QueueRxFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  uint8_t next = (uint8_t)((s_rx_head + 1U) % PROTOCOL_RX_QUEUE_LEN);
  if (next == s_rx_tail) {
    s_rx_dropped++;
    s_rx_overflow = true;
    s_comm_stats.rx_frames_dropped++;
    return false;
  }
  s_rx_queue[s_rx_head] = *frame;
  s_rx_head = next;
  s_comm_stats.rx_frames_total++;
  // Update queue depth statistics
  uint8_t depth = (s_rx_head >= s_rx_tail)
                      ? (s_rx_head - s_rx_tail)
                      : (PROTOCOL_RX_QUEUE_LEN - s_rx_tail + s_rx_head);
  if (depth > s_rx_queue_peak) {
    s_rx_queue_peak = depth;
  }
  return true;
}
/**
 * @brief CAN
 */
void Protocol_ProcessQueuedFrames(void) {
  CAN_Frame frame;
  if (s_rx_overflow) {
    s_rx_overflow = false;
    s_comm_stats.rx_overflow_events++;
    ErrorManager_Report(ERROR_COMM_INVALID_FRAME, "Rx queue overflow");
  }
  while (Protocol_DequeueRxFrame(&frame)) {
    Protocol_ProcessRxFrame(&frame);
  }
}
/**
 * @brief CAN
 * @note ，；
 *       ISR  Protocol_QueueRxFrame()
 */
void Protocol_ProcessRxFrame(const CAN_Frame *frame) {
  if (frame == NULL)
    return;
#ifdef DEBUG
  static uint32_t start_cnt = 0;
  uint32_t exec_time_us;
  start_cnt = DWT->CYCCNT;
#endif
  MotorCommand cmd;
  // 0. : INOVXIO (GET_ID)
  // ，motor
  if (s_current_protocol == PROTOCOL_INOVXIO) {
    uint8_t cmd_type = (frame->id >> 24) & 0x1F;
    if (cmd_type == INOVXIO_CMD_GET_ID) {
      uint8_t target = frame->id & 0xFF;
      // ID
      if (target == g_can_id || target == CAN_BROADCAST_ADDR) {
        CAN_Frame tx_frame;
        // : [0][MyID][FE][UUID-Low32]
        // UUID32，
        tx_frame.id = (0x00 << 24) | (g_can_id << 8) | 0xFE;
        tx_frame.is_extended = true;
        tx_frame.dlc = 8;
        // STM32ID (96-bit, factory programmed)
        DeviceUID_t uid;
        DeviceID_GetUID(&uid);
        memcpy(tx_frame.data, &uid, 8); // 8 (word0 + word1)
        Protocol_SendFrame(&tx_frame);
        return; // ，
      }
    }
  }
  // 1. : CAN
  ParseResult result = Protocol_ParseFrame(frame, &cmd);
  // 2. :  MotorCommand
  if (result == PARSE_OK) {
    Executor_ProcessCommand(&cmd);
  }
#ifdef DEBUG
  // Calculate execution time
  exec_time_us = (DWT->CYCCNT - start_cnt) / (SYS_CLOCK_HZ / 1000000UL);
  if (exec_time_us > s_comm_stats.exec_time_max_us) {
    s_comm_stats.exec_time_max_us = exec_time_us;
  }
#endif
}
/**
 * @brief  Callback for Safety Module to report faults via CAN.
 */
bool Protocol_ReportFaultCallback(uint32_t fault_bits, MOTOR_DATA *motor) {
  CAN_Frame tx_frame;
  // Build fault frame (CMD 21)
  if (Protocol_BuildFault(fault_bits, &tx_frame)) {
    // Send frame (returns true on success, false on TX full)
    return Protocol_SendFrame(&tx_frame);
  }
  return false;
}
void Protocol_PeriodicUpdate(uint32_t now_ms, const MotorStatus *status) {
  (void)status;
  if (s_current_protocol == PROTOCOL_CANOPEN) {
    CAN_Frame hb_frame;
    if (ProtocolCANopen_BuildHeartbeat(now_ms, &hb_frame)) {
      Protocol_SendFrame(&hb_frame);
    }
  }
}
/**
 * @brief Get communication statistics
 */
void Protocol_GetStats(CommStats_t *stats) {
  if (stats == NULL) {
    return;
  }
  __disable_irq();
  *stats = s_comm_stats;
  // Calculate current queue depth
  uint8_t depth = (s_rx_head >= s_rx_tail)
                      ? (s_rx_head - s_rx_tail)
                      : (PROTOCOL_RX_QUEUE_LEN - s_rx_tail + s_rx_head);
  stats->rx_queue_depth = depth;
  stats->rx_queue_peak = s_rx_queue_peak;
  __enable_irq();
}
/**
 * @brief Reset communication statistics
 */
void Protocol_ResetStats(void) {
  __disable_irq();
  s_comm_stats.rx_frames_total = 0;
  s_comm_stats.rx_frames_dropped = 0;
  s_comm_stats.rx_overflow_events = 0;
  s_comm_stats.tx_frames_total = 0;
  s_comm_stats.tx_frames_failed = 0;
  s_comm_stats.parse_errors = 0;
  s_comm_stats.exec_time_max_us = 0;
  s_rx_queue_peak = 0;
  __enable_irq();
}
