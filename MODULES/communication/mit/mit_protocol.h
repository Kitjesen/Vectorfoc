/**
 * @file    mit_protocol.h
 * @brief   MIT Cheetah protocol implementation.
 * @details
 * - Lightweight impedance control protocol from MIT.
 * - Units: Position [rad], Velocity [rad/s], Torque [Nm].
 * @version 1.0
 */

#ifndef PROTOCOL_MIT_H
#define PROTOCOL_MIT_H

#include "protocol_types.h"

/**
 * @brief MIT protocol command types.
 */
typedef enum {
  MIT_CMD_MOTOR_OFF = 0x00, /**< Motor disable */
  MIT_CMD_MOTOR_ON = 0x01,  /**< Motor enable */
  MIT_CMD_SET_ZERO = 0x02,  /**< Set zero position */
  MIT_CMD_CONTROL = 0x03,   /**< Impedance control command */
  MIT_CMD_GET_STATE = 0x04, /**< Get status */
} MITCmdType;

/* MIT protocol parameter ranges (Cheetah typical values) */
#define MIT_P_MIN -12.5f  /**< [rad] Position min */
#define MIT_P_MAX 12.5f   /**< [rad] Position max */
#define MIT_V_MIN -45.0f  /**< [rad/s] Velocity min */
#define MIT_V_MAX 45.0f   /**< [rad/s] Velocity max */
#define MIT_T_MIN -18.0f  /**< [Nm] Torque min */
#define MIT_T_MAX 18.0f   /**< [Nm] Torque max */
#define MIT_KP_MIN 0.0f   /**< Stiffness min */
#define MIT_KP_MAX 500.0f /**< Stiffness max */
#define MIT_KD_MIN 0.0f   /**< Damping min */
#define MIT_KD_MAX 5.0f   /**< Damping max */

/**
 * @brief  Initialize MIT protocol module.
 */
void ProtocolMIT_Init(void);

/**
 * @brief  Parse MIT protocol CAN frame.
 * @param  frame CAN frame.
 * @param  cmd   [out] Parsed command.
 * @return Parse result.
 */
ParseResult ProtocolMIT_Parse(const CAN_Frame *frame, MotorCommand *cmd);

/**
 * @brief  Build MIT feedback frame.
 * @param  status Motor status.
 * @param  frame  [out] CAN frame.
 * @return true on success, false on failure.
 */
bool ProtocolMIT_BuildFeedback(const MotorStatus *status, CAN_Frame *frame);

/**
 * @brief  Build MIT fault frame.
 * @param  fault_code Fault code.
 * @param  frame      [out] CAN frame.
 * @return true on success, false on failure.
 */
bool ProtocolMIT_BuildFault(uint32_t fault_code, CAN_Frame *frame);

#endif /* PROTOCOL_MIT_H */
