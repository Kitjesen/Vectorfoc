/**
 * @file    canopen_protocol.h
 * @brief   CANopen DS402 protocol implementation.
 * @details
 * - Standard industrial protocol for drives.
 * - Units: Position [counts], Velocity [counts/s], Torque [0.1%].
 * @version 1.0
 */

#ifndef PROTOCOL_CANOPEN_H
#define PROTOCOL_CANOPEN_H

#include "protocol_types.h"

/* CANopen Object Dictionary indices (DS402 standard) */
#define CANOPEN_OBJ_CONTROLWORD 0x6040     /**< Control word */
#define CANOPEN_OBJ_STATUSWORD 0x6041      /**< Status word */
#define CANOPEN_OBJ_MODES_OF_OP 0x6060     /**< Modes of operation */
#define CANOPEN_OBJ_TARGET_POSITION 0x607A /**< Target position */
#define CANOPEN_OBJ_TARGET_VELOCITY 0x60FF /**< Target velocity */
#define CANOPEN_OBJ_TARGET_TORQUE 0x6071   /**< Target torque */
#define CANOPEN_OBJ_POSITION_ACTUAL 0x6064 /**< Actual position */
#define CANOPEN_OBJ_VELOCITY_ACTUAL 0x606C /**< Actual velocity */
#define CANOPEN_OBJ_TORQUE_ACTUAL 0x6077   /**< Actual torque */

/**
 * @brief CANopen modes of operation.
 */
typedef enum {
  CANOPEN_MODE_POSITION = 1,     /**< Position mode */
  CANOPEN_MODE_VELOCITY = 2,     /**< Velocity mode */
  CANOPEN_MODE_TORQUE = 4,       /**< Torque mode */
  CANOPEN_MODE_HOMING = 6,       /**< Homing mode */
  CANOPEN_MODE_INTERPOLATED = 7, /**< Interpolated position mode */
  CANOPEN_MODE_CSP = 8,          /**< Cyclic synchronous position */
  CANOPEN_MODE_CSV = 9,          /**< Cyclic synchronous velocity */
  CANOPEN_MODE_CST = 10,         /**< Cyclic synchronous torque */
} CANopenMode;

/**
 * @brief CANopen node state (Heartbeat producer).
 */
typedef enum {
  CANOPEN_STATE_BOOTUP = 0x00,
  CANOPEN_STATE_STOPPED = 0x04,
  CANOPEN_STATE_OPERATIONAL = 0x05,
  CANOPEN_STATE_PREOPERATIONAL = 0x7F
} CANopenNodeState;

/**
 * @brief  Initialize CANopen protocol module.
 */
void ProtocolCANopen_Init(void);

/**
 * @brief  Parse CANopen CAN frame.
 * @param  frame CAN frame.
 * @param  cmd   [out] Parsed command.
 * @return Parse result.
 */
ParseResult ProtocolCANopen_Parse(const CAN_Frame *frame, MotorCommand *cmd);

/**
 * @brief  Build PDO feedback frame.
 * @param  status Motor status.
 * @param  frame  [out] CAN frame.
 * @return true on success, false on failure.
 */
bool ProtocolCANopen_BuildFeedback(const MotorStatus *status, CAN_Frame *frame);

/**
 * @brief  Build EMCY emergency frame.
 * @param  fault_code Fault code.
 * @param  frame      [out] CAN frame.
 * @return true on success, false on failure.
 */
bool ProtocolCANopen_BuildFault(uint32_t fault_code, CAN_Frame *frame);

/**
 * @brief Handle NMT command frame (COB-ID 0x000).
 * @param frame CAN frame (NMT).
 * @return true if handled, false if ignored.
 */
bool ProtocolCANopen_HandleNMT(const CAN_Frame *frame);

/**
 * @brief Build heartbeat frame if period elapsed.
 * @param now_ms Current system tick in ms.
 * @param frame  [out] CAN frame.
 * @return true if frame should be sent, false otherwise.
 */
bool ProtocolCANopen_BuildHeartbeat(uint32_t now_ms, CAN_Frame *frame);

#endif /* PROTOCOL_CANOPEN_H */
