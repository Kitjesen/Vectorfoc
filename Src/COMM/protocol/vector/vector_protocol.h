/**
 * @file    vector_protocol.h
 * @brief   Vector private CAN protocol
 * @details
 *          Units: position [rad], velocity [rad/s], torque [Nm], current [A]
 * @version 1.1
 */
#ifndef PROTOCOL_VECTOR_H
#define PROTOCOL_VECTOR_H
#include "protocol_types.h"
/**
 * @brief Vector protocol command types
 */
typedef enum {
  VECTOR_CMD_GET_ID = 0,              /**< Get device ID */
  VECTOR_CMD_MOTOR_CTRL = 1,          /**< Motor control (MIT impedance) */
  VECTOR_CMD_MOTOR_FEEDBACK = 2,      /**< Motor state feedback */
  VECTOR_CMD_MOTOR_ENABLE = 3,        /**< Enable motor */
  VECTOR_CMD_MOTOR_STOP = 4,          /**< Stop motor */
  VECTOR_CMD_SET_ZERO = 6,            /**< Set zero position */
  VECTOR_CMD_SET_ID = 7,              /**< Set CAN ID */
  VECTOR_CMD_CALIBRATE = 8,           /**< Trigger calibration */
  VECTOR_CMD_CALIB_STATUS = 9,        /**< Calibration status query/report */
  VECTOR_CMD_CALIB_ABORT = 10,        /**< Abort calibration */
  VECTOR_CMD_RESET = 0x0B,            /**< System reset (11) */
  VECTOR_CMD_CLEAR_FAULT = 0x0C,      /**< Clear fault (12) */
  VECTOR_CMD_BOOTLOADER = 0x0D,       /**< Enter bootloader (13) */
  VECTOR_CMD_CALIB_VALIDATE = 14,     /**< Pre-calibration prerequisite check */
  VECTOR_CMD_PARAM_READ = 17,         /**< Read parameter */
  VECTOR_CMD_PARAM_WRITE = 18,        /**< Write parameter */
  VECTOR_CMD_FAULT = 21,              /**< Fault notification */
  VECTOR_CMD_SAVE = 22,               /**< Save parameters to flash */
  VECTOR_CMD_SET_BAUDRATE = 23,       /**< Set CAN baudrate */
  VECTOR_CMD_REPORT = 24,             /**< Enable/disable periodic reporting */
  VECTOR_CMD_SET_PROTOCOL = 25,       /**< Switch protocol */
  VECTOR_CMD_GET_VERSION = 26,        /**< Get firmware version */
  VECTOR_CMD_FAULT_QUERY = 30,        /**< Query fault details */
} VectorCmdType;
/**
 * @brief  Initialize Vector protocol
 */
void ProtocolVector_Init(void);
/**
 * @brief   Parse a received CAN frame
 * @param  frame  CAN frame
 * @param  cmd    [out] parsed command
 * @return parse result
 */
ParseResult ProtocolVector_Parse(const CAN_Frame *frame, MotorCommand *cmd);
/**
 * @brief  Build motor feedback frame (CMD 0x02)
 * @param  status  motor status snapshot
 * @param  frame   [out] CAN frame
 * @return true on success
 */
bool ProtocolVector_BuildFeedback(const MotorStatus *status, CAN_Frame *frame);
/**
 * @brief  Build fault notification frame (CMD 0x15)
 * @param  fault_code   active fault bitmask
 * @param  warning_code active warning bitmask
 * @param  frame        [out] CAN frame
 * @return true on success
 */
bool ProtocolVector_BuildFault(uint32_t fault_code, uint32_t warning_code,
                               CAN_Frame *frame);
/**
 * @brief  Build parameter read/write response frame (CMD 0x12)
 * @param  param_index  parameter index
 * @param  value        parameter value
 * @param  frame        [out] CAN frame
 * @return true on success
 */
bool ProtocolVector_BuildParamResponse(uint16_t param_index, float value,
                                       CAN_Frame *frame);
/**
 * @brief  Build fault detail query response frame (CMD 0x1E)
 * @param  status  motor status snapshot
 * @param  frame   [out] CAN frame
 * @return true on success
 */
bool ProtocolVector_BuildFaultDetail(const MotorStatus *status,
                                     CAN_Frame *frame);
/**
 * @brief  Build calibration status frame (CMD 0x09)
 * @param  status  motor status with calibration fields populated
 * @param  frame   [out] CAN frame
 * @return true on success
 */
bool ProtocolVector_BuildCalibStatus(const MotorStatus *status,
                                     CAN_Frame *frame);
/**
 * @brief  Build pre-calibration validation response frame (CMD 0x0E)
 * @param  pass_mask  bitmask of passed checks
 * @param  fail_mask  bitmask of failed checks
 * @param  vbus       bus voltage [V]
 * @param  temp       motor temperature [degC]
 * @param  frame      [out] CAN frame
 * @return true on success
 */
bool ProtocolVector_BuildCalibValidate(uint8_t pass_mask, uint8_t fail_mask,
                                       float vbus, float temp,
                                       CAN_Frame *frame);
#endif /* PROTOCOL_VECTOR_H */
