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
 * @file    inovxio_protocol.h
 * @brief   Inovxio (MinerU/Robstride)
 * @details
 *          : position [rad], speed/velocity [rad/s],  [Nm], current [A]
 * @version 1.0
 */
#ifndef PROTOCOL_INOVXIO_H
#define PROTOCOL_INOVXIO_H
#include "protocol_types.h"
/**
 * @brief Inovxio
 */
typedef enum {
  PRIVATE_CMD_GET_ID = 0,         /**< getID */
  PRIVATE_CMD_MOTOR_CTRL = 1,     /**<  */
  PRIVATE_CMD_MOTOR_FEEDBACK = 2, /**< motorfeedback */
  PRIVATE_CMD_MOTOR_ENABLE = 3,   /**< motorenable */
  PRIVATE_CMD_MOTOR_STOP = 4,     /**< motorstop */
  PRIVATE_CMD_SET_ZERO = 6,       /**< set */
  PRIVATE_CMD_SET_ID = 7,         /**< setCAN ID */
  PRIVATE_CMD_CALIBRATE = 8,      /**< calibration */
  PRIVATE_CMD_RESET = 0x0B,       /**< reset (11) */
  PRIVATE_CMD_CLEAR_FAULT = 0x0C, /**< fault (12) */
  PRIVATE_CMD_BOOTLOADER = 0x0D,  /**< Bootloader (13) */
  PRIVATE_CMD_PARAM_READ = 17,    /**< param */
  PRIVATE_CMD_PARAM_WRITE = 18,   /**< param */
  PRIVATE_CMD_FAULT = 21,         /**< faultfeedback */
  PRIVATE_CMD_SAVE = 22,          /**< param */
  PRIVATE_CMD_SET_BAUDRATE = 23,  /**< set */
  PRIVATE_CMD_REPORT = 24,        /**<  */
  PRIVATE_CMD_SET_PROTOCOL = 25,  /**<  */
  PRIVATE_CMD_GET_VERSION = 26,   /**<  */
  PRIVATE_CMD_FAULT_QUERY = 30,   /**< fault */
} PrivateCmdType;
/**
 * @brief  init Inovxio
 */
void ProtocolPrivate_Init(void);
/**
 * @brief   Inovxio  CAN
 * @param  frame CAN
 * @param  cmd   [out]
 * @return
 */
ParseResult ProtocolPrivate_Parse(const CAN_Frame *frame, MotorCommand *cmd);
/**
 * @brief  feedback
 * @param  status motorstate
 * @param  frame  [out] CAN
 * @return  true,  false
 */
bool ProtocolPrivate_BuildFeedback(const MotorStatus *status, CAN_Frame *frame);
/**
 * @brief  fault
 * @param  fault_code   fault
 * @param  warning_code warning
 * @param  frame        [out] CAN
 * @return  true,  false
 */
bool ProtocolPrivate_BuildFault(uint32_t fault_code, uint32_t warning_code,
                                CAN_Frame *frame);
/**
 * @brief  param
 * @param  param_index param
 * @param  value       param
 * @param  frame       [out] CAN
 * @return  true,  false
 */
bool ProtocolPrivate_BuildParamResponse(uint16_t param_index, float value,
                                        CAN_Frame *frame);
/**
 * @brief  fault (CMD 30)
 * @param  status motorstate
 * @param  frame  [out] CAN
 * @return  true,  false
 */
bool ProtocolPrivate_BuildFaultDetail(const MotorStatus *status,
                                      CAN_Frame *frame);
#endif /* PROTOCOL_INOVXIO_H */
