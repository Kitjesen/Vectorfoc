/**
 * @file cmd_service.c
 * @brief  - CANstate
 * @note  cmd_task.c
 */
#include "cmd_service.h"
#include "bsp_log.h"
#include "calibration_context.h"
#include "manager.h"
#include "motor.h"
#include "config.h"
#include "safety_control.h"
#include "param_access.h"
#include "stm32g4xx_hal.h"
static bool s_report_enabled = false;
static inline void CmdService_SnapshotStatus(MotorStatus *status) {
  if (status == NULL)
    return;
  __disable_irq();
  status->can_id = g_can_id;
  status->position = motor_data.feedback.position;
  status->velocity = motor_data.feedback.velocity;
  status->current = motor_data.algo_output.Iq;
  status->torque =
      motor_data.algo_output.Iq * motor_data.Controller.torque_const;
  status->temperature = motor_data.feedback.temperature;
  status->voltage = motor_data.algo_input.Vbus;
  status->motor_state = motor_data.state.State_Mode;
  status->control_mode = motor_data.state.Control_Mode;
  status->fault_code = Safety_GetActiveFaultBits();
  // Calibration status fields
  status->calib_stage = motor_data.state.Sub_State;
  status->calib_sub_stage = motor_data.state.Cs_State;
  status->calib_progress = CalibContext_GetProgress(
      motor_data.state.Sub_State, motor_data.state.Cs_State,
      &motor_data.calib_ctx);
  status->calib_result = motor_data.last_calib_result;
  __enable_irq();
}
void CmdService_Init(void) { LOGINFO("[CMD] Command service initialized"); }
void CmdService_SetReportEnable(bool enable) { s_report_enabled = enable; }
void CmdService_Process(void) {
  static uint32_t last_report_time = 0;
  static bool last_fault_state = false;
  static float report_iq_filt = 0.0f;
  static float report_id_filt = 0.0f;
  static bool report_current_init = false;
  static uint8_t prev_calib_stage = 0; // SUB_STATE_IDLE
  uint32_t now = HAL_GetTick();
  // param
  Param_ProcessScheduledSave();
  // motorstate
  MotorStatus status;
  CmdService_SnapshotStatus(&status);
  Protocol_PeriodicUpdate(now, &status);
  bool has_fault = (status.fault_code != FAULT_NONE);
  // fault: fault
  if (has_fault && !last_fault_state) {
    CAN_Frame fault_frame;
    if (Protocol_BuildFault(status.fault_code, &fault_frame)) {
      Protocol_SendFrame(&fault_frame);
    }
  }
  last_fault_state = has_fault;
  // Auto-push calibration status frame when calibration stage changes
  if (status.calib_stage != prev_calib_stage) {
    prev_calib_stage = status.calib_stage;
    CAN_Frame calib_frame;
    if (Protocol_BuildCalibStatus(&status, &calib_frame)) {
      Protocol_SendFrame(&calib_frame);
    }
  }
  // statefeedback: fault (100Hz)
  if (s_report_enabled && !has_fault) {
    if (now - last_report_time >= 10) { // 100Hz
      float dt =
          (last_report_time == 0) ? 0.0f : (now - last_report_time) * 0.001f;
      last_report_time = now;
      float iq_raw = motor_data.algo_output.Iq;
      float id_raw = motor_data.algo_output.Id;
      if (!report_current_init || dt <= 0.0f) {
        report_iq_filt = iq_raw;
        report_id_filt = id_raw;
        report_current_init = true;
      } else {
        float alpha = 1.0f;
        if (REPORT_CURRENT_FILTER_FC > 0.0f) {
          float omega = 2.0f * M_PI * REPORT_CURRENT_FILTER_FC;
          alpha = (omega * dt) / (1.0f + omega * dt);
        }
        report_iq_filt += alpha * (iq_raw - report_iq_filt);
        report_id_filt += alpha * (id_raw - report_id_filt);
      }
      status.current = report_iq_filt;
      status.torque = report_iq_filt * motor_data.Controller.torque_const;
      CAN_Frame tx_frame;
      if (Protocol_BuildFeedback(&status, &tx_frame)) {
        Protocol_SendFrame(&tx_frame);
      }
    }
  }
}
