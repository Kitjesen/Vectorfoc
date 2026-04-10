#include "inovxio_protocol.h"
#include "calibration_context.h"
#include "fault_def.h"
#include "manager.h" // For Protocol_SendFrame
#include "motor.h"
#include "param_access.h"
#include "protocol_types.h"
#include "rtos/cmd_service.h" // For CmdService_SetReportEnable
#include "safety_control.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
// ==========  ==========
// ID: [28:24] Cmd, [23:8] Data/MasterID, [7:0] TargetID
#define GET_CMD_TYPE(id) (((id) >> 24) & 0x1F)
#define GET_TARGET_ID(id) ((id) & 0xFF)
#define GET_ID_DATA(id) (((id) >> 8) & 0xFFFF)
#define BOOTLOADER_ADDR 0x1FFF0000U
static void JumpToBootloader(void) {
  uint32_t sp = *(__IO uint32_t *)(BOOTLOADER_ADDR);
  uint32_t reset = *(__IO uint32_t *)(BOOTLOADER_ADDR + 4U);
  if (sp < 0x20000000U) {
    return;
  }
  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  for (uint32_t i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  __set_MSP(sp);
  __DSB();
  __ISB();
  ((void (*)(void))reset)();
}
// ==========  ==========
/**
 * @brief Uint16  Float ()
 */
static float Uint16ToFloat(uint16_t x, float min_val, float max_val) {
  float span = max_val - min_val;
  return ((float)x / 65535.0f) * span + min_val;
}
/**
 * @brief Float  Uint16 ()
 */
static uint16_t FloatToUint16(float x, float min_val, float max_val) {
  float span = max_val - min_val;
  if (x < min_val)
    x = min_val;
  if (x > max_val)
    x = max_val;
  return (uint16_t)(((x - min_val) / span) * 65535.0f);
}
/**
 * @brief 16 (, )
 */
static uint16_t BufToUint16(const uint8_t *val_ptr) {
  return (uint16_t)((val_ptr[0] << 8) | val_ptr[1]);
}
/**
 * @brief 16
 */
static void Uint16ToBuf(uint16_t val, uint8_t *val_ptr) {
  val_ptr[0] = (val >> 8) & 0xFF;
  val_ptr[1] = val & 0xFF;
}
/**
 * @brief mode ( 1)
 * @details ID Data:  (0~65535 -> -120~120Nm)
 *          Payload: [PosH][PosL][VelH][VelL][KpH][KpL][KdH][KdL]
 */
static ParseResult ParseMotorCtrl(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame->dlc < 8)
    return PARSE_ERR_INVALID_FRAME;
  // 1.  ID
  uint16_t torque_raw = GET_ID_DATA(frame->id);
  cmd->torque_ff = Uint16ToFloat(torque_raw, -120.0f, 120.0f);
  // 2. param ()
  uint16_t pos_raw = BufToUint16(&frame->data[0]);
  uint16_t vel_raw = BufToUint16(&frame->data[2]);
  uint16_t kp_raw = BufToUint16(&frame->data[4]);
  uint16_t kd_raw = BufToUint16(&frame->data[6]);
  // 3.
  cmd->pos_setpoint = Uint16ToFloat(pos_raw, -12.57f, 12.57f); // Pos: ±4π rad
  cmd->vel_setpoint = Uint16ToFloat(vel_raw, -15.0f, 15.0f); // Vel: ±15.0 rad/s
  cmd->kp = Uint16ToFloat(kp_raw, 0.0f, 500.0f); // Kp: 0~500 ()
  cmd->kd = Uint16ToFloat(kd_raw, 0.0f, 100.0f); // Kd: 0~100
  cmd->control_mode = CONTROL_MODE_MIT;
  cmd->enable_motor = true;
  return PARSE_OK;
}
/**
 * @brief param ( 17/18)
 * @details : [IdxL][IdxH][Pad][Pad][Val0][Val1][Val2][Val3]
 */
static ParseResult ParseParamCommand(const CAN_Frame *frame, MotorCommand *cmd,
                                     bool is_write) {
  if (frame->dlc < 4)
    return PARSE_ERR_INVALID_FRAME;
  // param ()
  cmd->param_index = (uint16_t)(frame->data[0] | (frame->data[1] << 8));
  if (is_write) {
    if (frame->dlc < 8)
      return PARSE_ERR_INVALID_FRAME;
    // param (4 )
    memcpy(&cmd->param_value, &frame->data[4], 4);
    cmd->is_param_write = true;
  } else {
    cmd->is_param_read = true;
  }
  return PARSE_OK;
}
/**
 * @brief
 */
ParseResult ProtocolPrivate_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  if (!frame || !cmd)
    return PARSE_ERR_INVALID_FRAME;
  // 1. check
  if (!frame->is_extended)
    return PARSE_ERR_INVALID_FRAME;
  memset(cmd, 0, sizeof(MotorCommand));
  PrivateCmdType type = (PrivateCmdType)GET_CMD_TYPE(frame->id);
  switch (type) {
  case PRIVATE_CMD_MOTOR_CTRL: // 1
    return ParseMotorCtrl(frame, cmd);
  case PRIVATE_CMD_MOTOR_ENABLE: // 3
    cmd->enable_motor = true;
    return PARSE_OK;
  case PRIVATE_CMD_MOTOR_STOP: // 4
    cmd->enable_motor = false;
    return PARSE_OK;
  case PRIVATE_CMD_SET_ZERO: // 6
    cmd->set_zero = true;
    return PARSE_OK;
  case PRIVATE_CMD_CALIBRATE: // 8
    {
      uint8_t type = 0;
      if (frame->dlc >= 1)
        type = frame->data[0];
      Motor_RequestCalibration(&motor_data, type);
    }
    return PARSE_OK;
  case PRIVATE_CMD_CALIB_STATUS: { // 9 — query calibration progress
    MotorStatus s;
    s.can_id = g_can_id;
    s.calib_stage = motor_data.state.Sub_State;
    s.calib_sub_stage = motor_data.state.Cs_State;
    s.calib_progress = CalibContext_GetProgress(
        motor_data.state.Sub_State, motor_data.state.Cs_State,
        &motor_data.calib_ctx);
    s.calib_result = motor_data.last_calib_result;
    CAN_Frame rsp;
    if (ProtocolPrivate_BuildCalibStatus(&s, &rsp))
      Protocol_SendFrame(&rsp);
    return PARSE_OK;
  }
  case PRIVATE_CMD_CALIB_ABORT: // 10 — abort calibration
    Motor_AbortCalibration(&motor_data);
    {
      MotorStatus s;
      s.can_id = g_can_id;
      s.calib_stage = motor_data.state.Sub_State;
      s.calib_sub_stage = motor_data.state.Cs_State;
      s.calib_progress = 0;
      s.calib_result = motor_data.last_calib_result;
      CAN_Frame rsp;
      if (ProtocolPrivate_BuildCalibStatus(&s, &rsp))
        Protocol_SendFrame(&rsp);
    }
    return PARSE_OK;
  case PRIVATE_CMD_CALIB_VALIDATE: { // 14 — pre-calibration check
    uint8_t fail_mask = 0;
    uint8_t pass_mask = Motor_PreCalibCheck(&motor_data, &fail_mask);
    CAN_Frame rsp;
    if (ProtocolPrivate_BuildCalibValidate(
            pass_mask, fail_mask, motor_data.algo_input.Vbus,
            motor_data.feedback.temperature, &rsp))
      Protocol_SendFrame(&rsp);
    return PARSE_OK;
  }
  case PRIVATE_CMD_SET_ID: // 7
    //  Param API  ID
    if (frame->dlc >= 4) { // Assumed layout: [NewID][0][0][0]
      // ID?
      //  MasterID 1?
      // : Byte0 = NewID
      float new_id = (float)frame->data[0];
      Param_WriteFloat(PARAM_CAN_ID, new_id);
      Param_ScheduleSave(); // （ISRsafety）
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;
  case PRIVATE_CMD_PARAM_READ: // 17
    return ParseParamCommand(frame, cmd, false);
  case PRIVATE_CMD_PARAM_WRITE: // 18
    return ParseParamCommand(frame, cmd, true);
  case PRIVATE_CMD_SAVE: // 22
    Param_ScheduleSave();
    return PARSE_OK;
  case PRIVATE_CMD_GET_VERSION: // 26
    //
    // : [Major][Minor][Patch][0][BuiltYear][Month][Day][0]
    // : 1.0.0
    {
      CAN_Frame tx_frame;
      tx_frame.id = (0x1A << 24) | (g_can_id << 8) | 0xFD;
      tx_frame.is_extended = true;
      tx_frame.dlc = 8;
      tx_frame.data[0] = 1; // Major
      tx_frame.data[1] = 0; // Minor
      tx_frame.data[2] = 0; // Patch
      tx_frame.data[3] = 0;
      Protocol_SendFrame(&tx_frame);
    }
    return PARSE_OK;
  case PRIVATE_CMD_FAULT_QUERY: // 30
    cmd->is_fault_query = true;
    return PARSE_OK;
  case PRIVATE_CMD_RESET: // 11
    HAL_NVIC_SystemReset();
    return PARSE_OK;
  case PRIVATE_CMD_CLEAR_FAULT: // 12
    Motor_ClearFaults(&motor_data);
    return PARSE_OK;
  case PRIVATE_CMD_BOOTLOADER: // 13 - Bootloader
    JumpToBootloader();
    return PARSE_OK;
    // ， OK
  case PRIVATE_CMD_REPORT: // 24
    if (frame->dlc >= 1) {
      // 0: Off, 1: On
      CmdService_SetReportEnable(frame->data[0] != 0);
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;
  case PRIVATE_CMD_SET_BAUDRATE: // 23
  case PRIVATE_CMD_SET_PROTOCOL: // 25
    if (frame->dlc >= 1) {
      cmd->is_protocol_switch = true;
      cmd->target_protocol = frame->data[0];
      return PARSE_OK;
    }
    return PARSE_ERR_INVALID_FRAME;
  default:
    return PARSE_ERR_UNSUPPORTED;
  }
}
/**
 * @brief motorfeedback ( 2)
 * @details ID: [Cmd(5)][Mode(2)+Fault(6)+MasterID(8)][MyID(8)][Host(8)]
 */
bool ProtocolPrivate_BuildFeedback(const MotorStatus *status,
                                   CAN_Frame *frame) {
  if (!status || !frame)
    return false;
  // ID (Bit 23-8)
  // MinerU MD:
  // Bit 23-22: Mode (0:Reset, 1:Cali, 2:Motor)
  // Bit 21-16: Fault bits (6 bits)
  // Bit 15-8:  Master ID (0xFD)
  // Mode: 1 = Calibrating, 2 = Motor operational
  uint32_t mode = (status->calib_stage != 0u) ? 1u : 2u;
  // fault (Bit 21-16)
  // Bit 21:  (FAULT_ENCODER_UNCALIBRATED)
  // Bit 20: overload (FAULT_STALL_OVERLOAD)
  // Bit 19: fault (FAULT_HARDWARE_ID like)
  // Bit 18:  (FAULT_OVER_TEMP)
  // Bit 17: phasecurrentfault (FAULT_CURRENT_A/B/C)
  // Bit 16: fault (FAULT_UNDER_VOLTAGE)
  uint32_t fault_bits = 0;
  if (status->fault_code & FAULT_ENCODER_UNCALIBRATED)
    fault_bits |= (1 << 5); // Bit 21
  if (status->fault_code & FAULT_STALL_OVERLOAD)
    fault_bits |= (1 << 4); // Bit 20
  if (status->fault_code & FAULT_HARDWARE_ID)
    fault_bits |= (1 << 3); // Bit 19
  if (status->fault_code & FAULT_OVER_TEMP)
    fault_bits |= (1 << 2); // Bit 18
  if (status->fault_code &
      (FAULT_CURRENT_A | FAULT_CURRENT_B | FAULT_CURRENT_C))
    fault_bits |= (1 << 1); // Bit 17
  if (status->fault_code & FAULT_UNDER_VOLTAGE)
    fault_bits |= (1 << 0); // Bit 16
  // ID: [Cmd 5bit][Info 16bit][Target 8bit]
  // Cmd: 0x02
  // Info: Mode(2) + Faults(6) + Master(8)
  // Target: HostID (0xFF/0xFD?) -> MD Feedback Frame defines Target as Host ID
  uint32_t id = (0x02 << 24) | (mode << 22) | (fault_bits << 16) |
                (status->can_id << 8) | 0xFD; // Host ID
  frame->id = id;
  frame->is_extended = true;
  frame->dlc = 8;
  // ========== : [position][speed/velocity][][temperature] ==========
  uint16_t pos = FloatToUint16(status->position, -12.57f, 12.57f);
  uint16_t vel = FloatToUint16(status->velocity, -15.0f, 15.0f);
  uint16_t tor = FloatToUint16(status->torque, -120.0f, 120.0f);
  int16_t temp = (int16_t)(status->temperature * 10.0f); // temperature × 10
  Uint16ToBuf(pos, &frame->data[0]);
  Uint16ToBuf(vel, &frame->data[2]);
  Uint16ToBuf(tor, &frame->data[4]);
  Uint16ToBuf((uint16_t)temp, &frame->data[6]);
  return true;
}
/**
 * @brief fault ( 21)
 */
bool ProtocolPrivate_BuildFault(uint32_t fault_code, uint32_t warning_code,
                                CAN_Frame *frame) {
  if (!frame)
    return false;
  //  21: fault
  frame->id = (0x15 << 24) | (g_can_id << 8) | 0xFD;
  frame->is_extended = true;
  frame->dlc = 8;
  // fault (4 , )
  frame->data[0] = (fault_code >> 24) & 0xFF;
  frame->data[1] = (fault_code >> 16) & 0xFF;
  frame->data[2] = (fault_code >> 8) & 0xFF;
  frame->data[3] = fault_code & 0xFF;
  return true;
}
/**
 * @brief init Inovxio
 */
void ProtocolPrivate_Init(void) {
  // init
}
/**
 * @brief param ( 18 )
 */
bool ProtocolPrivate_BuildParamResponse(uint16_t param_index, float value,
                                        CAN_Frame *frame) {
  if (!frame)
    return false;
  //  18 : param
  frame->id = (0x12 << 24) | (g_can_id << 8) | 0xFD;
  frame->is_extended = true;
  frame->dlc = 8;
  // param ()
  frame->data[0] = param_index & 0xFF;
  frame->data[1] = (param_index >> 8) & 0xFF;
  frame->data[2] = 0;
  frame->data[3] = 0;
  // param (4 )
  memcpy(&frame->data[4], &value, 4);
  return true;
}
/**
 * @brief fault ( 30)
 * @param status motorstate
 * @param frame [out] CAN
 * @return  true,  false
 */
bool ProtocolPrivate_BuildFaultDetail(const MotorStatus *status,
                                      CAN_Frame *frame) {
  if (!status || !frame)
    return false;
  // CMD 30: [CMD 30][SourceID][TargetID]
  frame->id = (0x1E << 24) | (status->can_id << 8) | 0xFD;
  frame->is_extended = true;
  frame->dlc = 8;
  // Data[0-1]: Fault Code (uint16, Big-Endian)
  frame->data[0] = (status->fault_code >> 8) & 0xFF;
  frame->data[1] = (status->fault_code) & 0xFF;
  // Data[2-3]: Fault Code High (Reserved, 0)
  frame->data[2] = (status->fault_code >> 24) & 0xFF;
  frame->data[3] = (status->fault_code >> 16) &
                   0xFF; // Not strictly big-endian logic for reserved, but
                         // consistent with 32-bit split
  // Data[4-7]: Fault Timestamp (uint32, Big-Endian)
  uint32_t timestamp = Safety_GetLastFaultTime();
  frame->data[4] = (timestamp >> 24) & 0xFF;
  frame->data[5] = (timestamp >> 16) & 0xFF;
  frame->data[6] = (timestamp >> 8) & 0xFF;
  frame->data[7] = timestamp & 0xFF;
  return true;
}
/**
 * @brief  Build calibration status frame (CMD 0x09)
 * @details
 *   CAN_ID: [0x09<<24] | [can_id<<8] | 0xFD
 *   data[0] = sub_state   (SUB_STATE: 0=IDLE,1=CURRENT,2=RSLS,3=FLUX)
 *   data[1] = cs_state    (CS_STATE: detailed sub-stage within RSLS)
 *   data[2] = progress    (0-100%)
 *   data[3] = last_result (CalibResult)
 *   data[4:5] = Rs × 1000 as uint16 (mΩ), valid after calibration
 *   data[6:7] = pole_pairs as uint16
 */
bool ProtocolPrivate_BuildCalibStatus(const MotorStatus *status,
                                      CAN_Frame *frame) {
  if (!status || !frame)
    return false;
  frame->id = (0x09u << 24) | ((uint32_t)status->can_id << 8) | 0xFDu;
  frame->is_extended = true;
  frame->dlc = 8;
  frame->data[0] = status->calib_stage;
  frame->data[1] = status->calib_sub_stage;
  frame->data[2] = status->calib_progress;
  frame->data[3] = status->calib_result;
  // Encode measured Rs (Ohm → mΩ as uint16, clamped to 0-65535)
  float rs_mohm = motor_data.parameters.Rs * 1000.0f;
  if (rs_mohm < 0.0f) rs_mohm = 0.0f;
  if (rs_mohm > 65535.0f) rs_mohm = 65535.0f;
  uint16_t rs_raw = (uint16_t)rs_mohm;
  Uint16ToBuf(rs_raw, &frame->data[4]);
  // Encode pole pairs
  uint16_t pp = (uint16_t)motor_data.parameters.pole_pairs;
  Uint16ToBuf(pp, &frame->data[6]);
  return true;
}
/**
 * @brief  Build pre-calibration validation response frame (CMD 0x0E)
 * @details
 *   CAN_ID: [0x0E<<24] | [can_id<<8] | 0xFD
 *   data[0] = pass_mask  (bit0=voltage, bit1=temp, bit2=motor_state, bit3=encoder)
 *   data[1] = fail_mask  (same bit layout)
 *   data[2:3] = Vbus × 100 as uint16
 *   data[4]   = temperature (uint8, °C)
 *   data[5:7] = reserved (0)
 */
bool ProtocolPrivate_BuildCalibValidate(uint8_t pass_mask, uint8_t fail_mask,
                                        float vbus, float temp,
                                        CAN_Frame *frame) {
  if (!frame)
    return false;
  frame->id = (0x0Eu << 24) | ((uint32_t)g_can_id << 8) | 0xFDu;
  frame->is_extended = true;
  frame->dlc = 8;
  frame->data[0] = pass_mask;
  frame->data[1] = fail_mask;
  float vbus_x100 = vbus * 100.0f;
  if (vbus_x100 < 0.0f) vbus_x100 = 0.0f;
  if (vbus_x100 > 65535.0f) vbus_x100 = 65535.0f;
  uint16_t vbus_raw = (uint16_t)vbus_x100;
  Uint16ToBuf(vbus_raw, &frame->data[2]);
  int16_t temp_raw = (int16_t)temp;
  frame->data[4] = (uint8_t)(temp_raw < 0 ? 0 : (temp_raw > 255 ? 255 : temp_raw));
  frame->data[5] = 0;
  frame->data[6] = 0;
  frame->data[7] = 0;
  return true;
}
