// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file manager.c
 * @brief  -
 *
 * :
 *   1. : Vector/CANopen/MIT
 *   2. :
 *   3. : motor
 *
 * :
 *   CAN → Protocol_ProcessRxFrame() →  → motor
 *                                       ↓
 *   CAN ← Protocol_BuildFeedback() ← state
 *
 * :
 *   Protocol_Init(PROTOCOL_VECTOR);        // init
 *   Protocol_ProcessRxFrame(&can_frame);    // CANinterrupt
 *   Protocol_SendFrame(&can_frame);         // CAN
    CAN 。
 */
#include "manager.h"
#include "bsp_dwt.h"
#include "device_id.h"
#include "error_manager.h"
#include "error_types.h"
#include "executor/executor.h"
#include "fault_detection.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"
#include "main.h" // For FDCAN support
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "platform.h"
#include "protocol/canopen/canopen_protocol.h"
#include "protocol/mit/mit_protocol.h"
#include "protocol/vector/vector_protocol.h"
#include "safety_control.h" // For Safety_GetActiveFaultBits
#include "transport.h"
#include <string.h>
#define CAN_BROADCAST_ADDR 0x7F   ///< CAN
#define VECTOR_CMD_GET_ID 0x00    ///< Vector GET_ID
#define PROTOCOL_RX_QUEUE_LEN 32U ///< Rx ring buffer length
/*  */
static ProtocolType s_current_protocol = PROTOCOL_VECTOR;
/* Active transport adapter; communication is disabled until one is bound. */
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
static uint32_t s_rx_overflow_reported_drops = 0;

static void Protocol_FillMotorStatus(MotorStatus *status) {
  if (status == NULL) {
    return;
  }
  memset(status, 0, sizeof(*status));
  status->position = Protocol_TurnsToRadians(motor_data.feedback.position);
  status->velocity = Protocol_TurnsToRadians(motor_data.feedback.velocity);
  status->torque = motor_data.Controller.torque_setpoint;
  status->temperature = motor_data.feedback.temperature;
  status->voltage = motor_data.algo_input.Vbus;
  status->motor_state = (uint8_t)motor_data.state.State_Mode;
  status->control_mode = (uint8_t)motor_data.state.Control_Mode;
  status->fault_code = Safety_GetActiveFaultBits();
  status->can_id = g_can_id;
  status->calib_stage = (uint8_t)motor_data.state.Sub_State;
  status->calib_sub_stage = (uint8_t)motor_data.state.Cs_State;
  status->calib_progress = CalibContext_GetProgress(motor_data.state.Sub_State,
                                                    motor_data.state.Cs_State,
                                                    &motor_data.calib_ctx);
  status->calib_result = motor_data.last_calib_result;
}

static bool Protocol_DequeueRxFrame(CAN_Frame *out) {
  if (out == NULL) {
    return false;
  }
  bool has_frame = false;
  CRITICAL_SECTION_BEGIN();
  if (s_rx_head != s_rx_tail) {
    *out = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % PROTOCOL_RX_QUEUE_LEN);
    has_frame = true;
  }
  CRITICAL_SECTION_END();
  return has_frame;
}
/**
 * @brief
 */
static void Protocol_ReceiveTransportFrame(const TransportFrame *frame) {
  if (frame == NULL || frame->type != TRANSPORT_CAN ||
      frame->len > sizeof(((CAN_Frame *)0)->data)) {
    return;
  }
  const uint32_t max_id = frame->is_extended ? 0x1FFFFFFFU : 0x7FFU;
  if (frame->id > max_id) {
    return;
  }

  CAN_Frame can_frame;
  Transport_ToCANFrame(frame, &can_frame);
  (void)Protocol_QueueRxFrame(&can_frame);
}

static void Protocol_UnbindTransport(void) {
  if (s_transport != NULL && s_transport->register_rx_callback != NULL) {
    (void)s_transport->register_rx_callback(NULL);
  }
  s_transport = NULL;
}

bool Protocol_RegisterTransport(const TransportInterface *transport) {
  Protocol_UnbindTransport();
  if (transport == NULL) {
    return false;
  }
  if (transport->type != TRANSPORT_CAN || transport->send == NULL ||
      transport->register_rx_callback == NULL) {
    return false;
  }
  if (!transport->register_rx_callback(Protocol_ReceiveTransportFrame)) {
    return false;
  }
  s_transport = transport;
  return true;
}
static const TransportInterface *Protocol_GetBoundTransport(void) {
  const TransportInterface *transport;
  CRITICAL_SECTION_BEGIN();
  transport = s_transport;
  CRITICAL_SECTION_END();
  return transport;
}

static void Protocol_RecordTxResult(bool success) {
  CRITICAL_SECTION_BEGIN();
  if (success) {
    s_comm_stats.tx_frames_total++;
  } else {
    s_comm_stats.tx_frames_failed++;
  }
  CRITICAL_SECTION_END();
}
/**
 * @brief init
 */
void Protocol_Init(ProtocolType default_protocol) {
  s_current_protocol = default_protocol;
  // init
  switch (s_current_protocol) {
  case PROTOCOL_VECTOR:
    ProtocolVector_Init();
    break;
  case PROTOCOL_CANOPEN:
    ProtocolCANopen_Init();
    break;
  case PROTOCOL_MIT:
    ProtocolMIT_Init();
    break;
  default:
    // Vector private protocol
    s_current_protocol = PROTOCOL_VECTOR;
    ProtocolVector_Init();
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
  if (frame->is_rtr) {
    s_comm_stats.parse_errors++;
    ERROR_REPORT(ERROR_COMM_INVALID_FRAME, "RTR frame rejected");
    return PARSE_ERR_INVALID_FRAME;
  }
  ParseResult result;
  switch (s_current_protocol) {
  case PROTOCOL_VECTOR:
    result = ProtocolVector_Parse(frame, cmd);
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
  case PROTOCOL_VECTOR:
    return ProtocolVector_BuildFeedback(status, frame);
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
  case PROTOCOL_VECTOR:
    return ProtocolVector_BuildFault(fault_code, 0, frame);
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
  case PROTOCOL_VECTOR:
    return ProtocolVector_BuildParamResponse(param_index, value, frame);
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
  case PROTOCOL_VECTOR:
    return ProtocolVector_BuildCalibStatus(status, frame);
  default:
    return false;
  }
}
/**
 * @brief CAN
 */
bool Protocol_SendFrame(const CAN_Frame *frame) {
  if (frame == NULL || frame->dlc > sizeof(frame->data) || frame->is_rtr) {
    return false;
  }

  bool result = false;
  const TransportInterface *transport = Protocol_GetBoundTransport();
  if (transport != NULL && transport->send != NULL) {
    TransportFrame transport_frame;
    Transport_FromCANFrame(frame, &transport_frame);
    result = transport->send(&transport_frame);
  }
  Protocol_RecordTxResult(result);
  return result;
}

bool Protocol_SendTrackedFrame(const CAN_Frame *frame,
                               TransportTxTicket *ticket) {
  if (frame == NULL || ticket == NULL) {
    return false;
  }
  memset(ticket, 0, sizeof(*ticket));
  if (frame->dlc > sizeof(frame->data) || frame->is_rtr) {
    return false;
  }

  bool result = false;
  const TransportInterface *transport = Protocol_GetBoundTransport();
  if (transport != NULL && transport->send_tracked != NULL) {
    TransportFrame transport_frame;
    Transport_FromCANFrame(frame, &transport_frame);
    result = transport->send_tracked(&transport_frame, ticket);
  }
  Protocol_RecordTxResult(result);
  return result;
}

bool Protocol_TxTicketIsComplete(const TransportTxTicket *ticket) {
  if (ticket == NULL || ticket->marker == 0U) {
    return false;
  }

  const TransportInterface *transport = Protocol_GetBoundTransport();
  return transport != NULL && transport->tx_ticket_is_complete != NULL &&
         transport->tx_ticket_is_complete(ticket);
}

void Protocol_CancelTrackedSend(const TransportTxTicket *ticket) {
  if (ticket == NULL || ticket->marker == 0U) {
    return;
  }

  const TransportInterface *transport = Protocol_GetBoundTransport();
  if (transport != NULL && transport->cancel_tracked_tx != NULL) {
    transport->cancel_tracked_tx(ticket);
  }
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
  CAN_Frame frame = {0};
  bool had_overflow = false;
  uint32_t dropped_snapshot = 0;
  CRITICAL_SECTION_BEGIN();
  had_overflow = s_rx_overflow;
  if (had_overflow) {
    s_rx_overflow = false;
    dropped_snapshot = s_rx_dropped;
  }
  CRITICAL_SECTION_END();
  if (had_overflow && dropped_snapshot != s_rx_overflow_reported_drops) {
    s_rx_overflow_reported_drops = dropped_snapshot;
    s_comm_stats.rx_overflow_events++;
    ErrorManager_Report(ERROR_COMM_RX_QUEUE_OVERFLOW, "Rx queue overflow");
  }
  while (Protocol_DequeueRxFrame(&frame)) {
    Protocol_ProcessRxFrame(&frame);
  }
  if (s_current_protocol == PROTOCOL_VECTOR) {
    ProtocolVector_Service();
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
  if (frame->is_rtr) {
    (void)Protocol_ParseFrame(frame, &cmd);
    return;
  }
  // 0. Vector GET_ID fast-path (before full parse)
  // ，motor
  if (s_current_protocol == PROTOCOL_VECTOR && frame->is_extended) {
    uint8_t cmd_type = (frame->id >> 24) & 0x1F;
    if (cmd_type == VECTOR_CMD_GET_ID) {
      uint8_t target = frame->id & 0xFF;
      // ID
      if (target == g_can_id || target == CAN_BROADCAST_ADDR) {
        CAN_Frame tx_frame = {0};
        // : [0][MyID][FE][UUID-Low32]
        // UUID32，
        tx_frame.id = (0x00 << 24) | (g_can_id << 8) | 0xFE;
        tx_frame.is_extended = true;
        tx_frame.dlc = 8;
        // STM32ID (96-bit, factory programmed)
        DeviceUID_t uid;
        DeviceID_GetUID(&uid);
        memcpy(tx_frame.data, &uid, 8); // 8 (word0 + word1)
        Detection_FeedWatchdog(HAL_GetSystemTick());
        Protocol_SendFrame(&tx_frame);
        return; // ，
      }
    }
  }
  // 1. : CAN
  ParseResult result = Protocol_ParseFrame(frame, &cmd);
  // 2. :  MotorCommand
  if (result == PARSE_OK) {
    Detection_FeedWatchdog(HAL_GetSystemTick());
    if (cmd.request_feedback) {
      MotorStatus status;
      CAN_Frame tx_frame = {0};
      Protocol_FillMotorStatus(&status);
      if (Protocol_BuildFeedback(&status, &tx_frame)) {
        Protocol_SendFrame(&tx_frame);
      }
    } else {
      Executor_ProcessCommand(&cmd);
    }
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
  CAN_Frame tx_frame = {0};
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
    CAN_Frame hb_frame = {0};
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
  CRITICAL_SECTION_BEGIN();
  *stats = s_comm_stats;
  // Calculate current queue depth
  uint8_t depth = (s_rx_head >= s_rx_tail)
                      ? (s_rx_head - s_rx_tail)
                      : (PROTOCOL_RX_QUEUE_LEN - s_rx_tail + s_rx_head);
  stats->rx_queue_depth = depth;
  stats->rx_queue_peak = s_rx_queue_peak;
  CRITICAL_SECTION_END();
}
/**
 * @brief Reset communication statistics
 */
void Protocol_ResetStats(void) {
  CRITICAL_SECTION_BEGIN();
  s_comm_stats.rx_frames_total = 0;
  s_comm_stats.rx_frames_dropped = 0;
  s_comm_stats.rx_overflow_events = 0;
  s_comm_stats.tx_frames_total = 0;
  s_comm_stats.tx_frames_failed = 0;
  s_comm_stats.parse_errors = 0;
  s_comm_stats.exec_time_max_us = 0;
  s_rx_queue_peak = 0;
  s_rx_overflow_reported_drops = s_rx_dropped;
  CRITICAL_SECTION_END();
}
