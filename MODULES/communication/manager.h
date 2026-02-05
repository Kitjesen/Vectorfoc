/**
 * @file    manager.h
 * @brief   Communication protocol manager - multi-protocol router.
 * @details
 * - Supported protocols: Inovxio (MinerU), CANopen DS402, MIT Cheetah.
 * - Context: Routes CAN frames to appropriate protocol handler.
 * - Thread safety: Rx can be queued from ISR; heavy processing should run in a
 *   task context via Protocol_ProcessQueuedFrames().
 *
 * Core API:
 * - Protocol_Init(): Initialize protocol
 * - Protocol_ParseFrame(): Parse CAN frame (route to protocol)
 * - Protocol_BuildFeedback(): Build feedback frame
 * - Protocol_ProcessRxFrame(): Complete processing (parse + execute + feedback)
 * - Protocol_QueueRxFrame(): Queue CAN frame from ISR
 * - Protocol_ProcessQueuedFrames(): Drain queued frames in task context
 */

#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include "protocol_types.h"

// Forward declaration to avoid circular dependency
typedef struct MOTOR_DATA_s MOTOR_DATA;

/**
 * @brief  Initialize protocol manager.
 * @param  default_protocol Default protocol type.
 */
void Protocol_Init(ProtocolType default_protocol);

/**
 * @brief  Set active protocol type.
 * @param  protocol Protocol type.
 */
void Protocol_SetType(ProtocolType protocol);

/**
 * @brief  Get current active protocol.
 * @return Current protocol type.
 */
ProtocolType Protocol_GetType(void);

/**
 * @brief  Parse CAN frame.
 * @param  frame CAN frame.
 * @param  cmd   [out] Parsed command.
 * @return Parse result.
 */
ParseResult Protocol_ParseFrame(const CAN_Frame *frame, MotorCommand *cmd);

/**
 * @brief  Build feedback frame.
 * @param  status Motor status.
 * @param  frame  [out] CAN frame.
 * @return true on success, false on failure.
 */
bool Protocol_BuildFeedback(const MotorStatus *status, CAN_Frame *frame);

/**
 * @brief  Build fault frame.
 * @param  fault_code Fault code.
 * @param  frame      [out] CAN frame.
 * @return true on success, false on failure.
 */
bool Protocol_BuildFault(uint32_t fault_code, CAN_Frame *frame);

/**
 * @brief  Build parameter response frame.
 * @param  param_index Parameter index.
 * @param  value       Parameter value.
 * @param  frame       [out] CAN frame.
 * @return true on success, false on failure.
 */
bool Protocol_BuildParamResponse(uint16_t param_index, float value,
                                 CAN_Frame *frame);

/**
 * @brief  Send CAN frame.
 * @param  frame CAN frame.
 * @return true on success, false on failure.
 */
bool Protocol_SendFrame(const CAN_Frame *frame);

/**
 * @brief  Process received CAN frame (integrated application logic).
 * @param  frame Received CAN frame.
 * @note   Call from task context; ISR should only call Protocol_QueueRxFrame().
 */
void Protocol_ProcessRxFrame(const CAN_Frame *frame);

/**
 * @brief  Queue received CAN frame from ISR (non-blocking).
 * @param  frame Received CAN frame.
 * @return true if queued, false if dropped.
 */
bool Protocol_QueueRxFrame(const CAN_Frame *frame);

/**
 * @brief  Process queued CAN frames in task context.
 * @note   Call from a periodic task (e.g. customTask).
 */
void Protocol_ProcessQueuedFrames(void);

/**
 * @brief  Callback for Safety Module to report faults via CAN.
 * @param  fault_bits Fault code.
 * @param  motor      Motor data pointer.
 * @return true on success, false on failure (needs retry).
 */
bool Protocol_ReportFaultCallback(uint32_t fault_bits, MOTOR_DATA *motor);

/**
 * @brief Periodic protocol maintenance (heartbeat, keepalive, etc).
 * @param now_ms Current system tick in ms.
 * @param status Motor status snapshot.
 */
void Protocol_PeriodicUpdate(uint32_t now_ms, const MotorStatus *status);

/**
 * @brief Communication statistics structure.
 */
typedef struct {
  uint32_t rx_frames_total;     ///< Total received frames
  uint32_t rx_frames_dropped;   ///< Frames dropped due to queue overflow
  uint32_t rx_queue_depth;      ///< Current queue depth
  uint32_t rx_queue_peak;       ///< Peak queue depth
  uint32_t rx_overflow_events;  ///< Number of overflow events
  uint32_t tx_frames_total;     ///< Total transmitted frames
  uint32_t tx_frames_failed;    ///< Failed transmissions
  uint32_t parse_errors;        ///< Parse errors
  uint32_t exec_time_max_us;    ///< Max frame processing time (microseconds)
} CommStats_t;

/**
 * @brief  Get communication statistics.
 * @param  stats [out] Statistics structure.
 */
void Protocol_GetStats(CommStats_t *stats);

/**
 * @brief  Reset communication statistics.
 */
void Protocol_ResetStats(void);

#endif /* COMM_MANAGER_H */
