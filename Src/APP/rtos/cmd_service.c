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
 * @file cmd_service.c
 * @brief  - CANstate
 * @note  cmd_task.c
 */
#include "cmd_service.h"
#include "bsp_log.h"
#include "calibration_context.h"
#include "error_manager.h"
#include "manager.h"
#include "motor.h"
#include "config.h"
#include "safety_control.h"
#include "param_access.h"
#include "platform.h"
#include "stm32g4xx_hal.h"
#include "vofa.h"
#define CMD_SERVICE_SAVE_RETRY_LIMIT 3U

static bool s_report_enabled = false;
static bool s_param_save_maintenance_reserved = false;
static bool s_param_save_maintenance_held = false;
static uint8_t s_param_save_attempts = 0U;
static uint32_t s_param_save_generation = 0U;

static inline void CmdService_SnapshotStatus(MotorStatus *status) {
  if (status == NULL)
    return;
  /* Safety owns its own critical section, so query it before taking ours. */
  uint32_t active_fault_bits = Safety_GetActiveFaultBits();
  CRITICAL_SECTION_BEGIN();
  status->can_id = g_can_id;
  status->position = Protocol_TurnsToRadians(motor_data.feedback.position);
  status->velocity = Protocol_TurnsToRadians(motor_data.feedback.velocity);
  status->current = motor_data.algo_output.Iq;
  status->torque =
      motor_data.algo_output.Iq * motor_data.Controller.torque_const;
  status->temperature = motor_data.feedback.temperature;
  status->voltage = motor_data.algo_input.Vbus;
  status->motor_state = motor_data.state.State_Mode;
  status->control_mode = motor_data.state.Control_Mode;
  status->fault_code = active_fault_bits;
  // Calibration status fields
  status->calib_stage = motor_data.state.Sub_State;
  status->calib_sub_stage = motor_data.state.Cs_State;
  status->calib_progress = CalibContext_GetProgress(
      motor_data.state.Sub_State, motor_data.state.Cs_State,
      &motor_data.calib_ctx);
  status->calib_result = motor_data.last_calib_result;
  CRITICAL_SECTION_END();
}

void CmdService_Init(void) { LOGINFO("[CMD] Command service initialized"); }
void CmdService_SetReportEnable(bool enable) { s_report_enabled = enable; }

bool CmdService_BeginScheduledSave(void) {
  if (!StateMachine_BeginMaintenance(&g_ds402_state_machine)) {
    return false;
  }
  CRITICAL_SECTION_BEGIN();
  s_param_save_maintenance_reserved = true;
  s_param_save_attempts = 0U;
  CRITICAL_SECTION_END();
  return true;
}

void CmdService_CommitScheduledSave(void) {
  Param_ScheduleSave();
  uint32_t generation = Param_GetScheduledSaveGeneration();
  CRITICAL_SECTION_BEGIN();
  s_param_save_maintenance_reserved = false;
  s_param_save_maintenance_held = true;
  s_param_save_generation = generation;
  CRITICAL_SECTION_END();
}

void CmdService_CancelScheduledSave(void) {
  CRITICAL_SECTION_BEGIN();
  s_param_save_maintenance_reserved = false;
  s_param_save_maintenance_held = false;
  s_param_save_attempts = 0U;
  s_param_save_generation = 0U;
  CRITICAL_SECTION_END();
  StateMachine_EndMaintenance(&g_ds402_state_machine);
}

bool CmdService_RequestScheduledSave(void) {
  if (!CmdService_BeginScheduledSave()) {
    return false;
  }
  CmdService_CommitScheduledSave();
  return true;
}

void CmdService_Process(void) {
  static uint32_t last_report_time = 0;
  static uint32_t last_calib_report_time = 0; // 1Hz progress report during calibration
  static bool last_fault_state = false;
  static float report_iq_filt = 0.0f;
  static float report_id_filt = 0.0f;
  static bool report_current_init = false;
  static uint8_t prev_calib_stage = 0; // SUB_STATE_IDLE
  static uint32_t next_param_save_attempt = 0;
  uint32_t now = HAL_GetTick();
  // param
  bool held_save = s_param_save_maintenance_held;
  bool process_save = held_save;
  bool release_maintenance = false;
  /* A newly committed external save already owns the maintenance lease.  Do
   * not make that lease wait behind the throttle from an unrelated previous
   * save; only retry attempts are rate limited. */
  bool initial_held_attempt = held_save && s_param_save_attempts == 0U;
  if (!process_save && Param_HasScheduledSave() &&
      (int32_t)(now - next_param_save_attempt) >= 0) {
    process_save = StateMachine_BeginMaintenance(&g_ds402_state_machine);
    release_maintenance = process_save;
    if (process_save) {
      uint32_t generation = Param_GetScheduledSaveGeneration();
      if (generation != s_param_save_generation) {
        s_param_save_attempts = 0U;
        s_param_save_generation = generation;
      }
    }
  }
  if (process_save &&
      (initial_held_attempt ||
       (int32_t)(now - next_param_save_attempt) >= 0)) {
    uint32_t generation = s_param_save_generation;
    bool save_succeeded = Param_ProcessScheduledSave();
    if (held_save) {
      Vofa_ReportScheduledSaveResult(save_succeeded);
    }
    if (save_succeeded ||
        ++s_param_save_attempts >= CMD_SERVICE_SAVE_RETRY_LIMIT) {
      if (!save_succeeded) {
        bool discarded = Param_DiscardScheduledSaveIfGeneration(generation);
        if (discarded && Param_RollbackScheduledSave() != PARAM_OK) {
          ErrorManager_Report(ERROR_PARAM_FLASH_WRITE,
                              "Scheduled save rollback failed");
        }
        if (held_save) {
          Vofa_ReportScheduledSaveFailed();
        }
      }
      CRITICAL_SECTION_BEGIN();
      s_param_save_maintenance_reserved = false;
      s_param_save_maintenance_held = false;
      s_param_save_attempts = 0U;
      s_param_save_generation = 0U;
      CRITICAL_SECTION_END();
      StateMachine_EndMaintenance(&g_ds402_state_machine);
    } else if (release_maintenance) {
      StateMachine_EndMaintenance(&g_ds402_state_machine);
    }
    next_param_save_attempt = now + 250U;
  }
  // motorstate
  MotorStatus status;
  CmdService_SnapshotStatus(&status);
  Protocol_PeriodicUpdate(now, &status);
  bool has_fault = (status.fault_code != FAULT_NONE);
  // fault: fault
  if (has_fault && !last_fault_state) {
    CAN_Frame fault_frame = {0};
    if (Protocol_BuildFault(status.fault_code, &fault_frame)) {
      Protocol_SendFrame(&fault_frame);
    }
  }
  last_fault_state = has_fault;
  // Auto-push calibration status frame on stage change (immediate)
  if (status.calib_stage != prev_calib_stage) {
    prev_calib_stage = status.calib_stage;
    last_calib_report_time = now; // align periodic timer to stage change
    CAN_Frame calib_frame = {0};
    if (Protocol_BuildCalibStatus(&status, &calib_frame)) {
      Protocol_SendFrame(&calib_frame);
    }
  }
  // Periodic calibration progress report at 1Hz while calibration is running
  // Reuses the same elapsed-time pattern as the 100Hz motor feedback below
  if (status.calib_stage != 0) {
    if (now - last_calib_report_time >= 1000u) { // 1 Hz
      last_calib_report_time = now;
      CAN_Frame calib_frame = {0};
      if (Protocol_BuildCalibStatus(&status, &calib_frame)) {
        Protocol_SendFrame(&calib_frame);
      }
    }
  } else {
    last_calib_report_time = 0u; // reset when idle so next calib starts fresh
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
      CAN_Frame tx_frame = {0};
      if (Protocol_BuildFeedback(&status, &tx_frame)) {
        Protocol_SendFrame(&tx_frame);
      }
    }
  }
}
