// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "cmd_service.h"
#include "error_manager.h"
#include "fsm.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "safety_control.h"
#include <stdio.h>
#include <string.h>

StateMachine g_ds402_state_machine;
MOTOR_DATA motor_data;
uint8_t g_can_id = 7U;
uint8_t g_can_baudrate = 1U;
uint8_t g_protocol_type = PROTOCOL_VECTOR;
uint32_t g_can_timeout_ms = 1000U;
uint8_t g_zero_sta = 0U;
float g_add_offset = 0.0f;
uint8_t g_damper_enable = 0U;
uint8_t g_run_mode = 1U;

static uint32_t s_tick;
static bool s_save_pending;
static uint32_t s_save_generation;
static unsigned s_save_process_count;
static unsigned s_rollback_count;
static unsigned s_discard_count;
static unsigned s_vofa_result_count;
static unsigned s_vofa_terminal_failed_count;
static unsigned s_error_report_count;

uint32_t HAL_GetTick(void) { return s_tick; }

bool StateMachine_BeginMaintenance(StateMachine *sm) {
  if (sm == NULL || sm->maintenance_active) {
    return false;
  }
  sm->maintenance_active = true;
  return true;
}

void StateMachine_EndMaintenance(StateMachine *sm) {
  if (sm != NULL) {
    sm->maintenance_active = false;
  }
}

uint8_t CalibContext_GetProgress(uint8_t sub_state, uint8_t cs_state,
                                 const CalibrationContext *ctx) {
  (void)sub_state;
  (void)cs_state;
  (void)ctx;
  return 0U;
}

uint32_t Safety_GetActiveFaultBits(void) { return 0U; }

void Protocol_PeriodicUpdate(uint32_t now_ms, const MotorStatus *status) {
  (void)now_ms;
  (void)status;
}

bool Protocol_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  (void)fault_code;
  (void)frame;
  return false;
}

bool Protocol_BuildCalibStatus(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}

bool Protocol_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}

bool Protocol_SendFrame(const CAN_Frame *frame) {
  (void)frame;
  return true;
}

void Vofa_ReportScheduledSaveResult(bool succeeded) {
  (void)succeeded;
  s_vofa_result_count++;
}

void Vofa_ReportScheduledSaveFailed(void) {
  s_vofa_terminal_failed_count++;
}

ParamResult Param_RollbackScheduledSave(void) {
  s_rollback_count++;
  return PARAM_OK;
}

void Param_ScheduleSave(void) {
  s_save_generation++;
  s_save_pending = true;
}

bool Param_HasScheduledSave(void) { return s_save_pending; }

uint32_t Param_GetScheduledSaveGeneration(void) { return s_save_generation; }

bool Param_DiscardScheduledSaveIfGeneration(uint32_t generation) {
  if (generation == s_save_generation) {
    s_save_pending = false;
    s_discard_count++;
    return true;
  }
  return false;
}

bool Param_ProcessScheduledSave(void) {
  if (!s_save_pending) {
    return false;
  }
  s_save_process_count++;
  s_save_pending = true;
  return false;
}

void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)code;
  (void)message;
  (void)file;
  (void)line;
  s_error_report_count++;
}

void ErrorManager_Report(uint32_t code, const char *message) {
  ErrorManager_ReportFull(code, message, __FILE__, __LINE__);
}

static void ResetState(void) {
  static uint32_t s_reset_tick_base = 1000U;
  memset(&g_ds402_state_machine, 0, sizeof(g_ds402_state_machine));
  memset(&motor_data, 0, sizeof(motor_data));
  motor_data.Controller.torque_const = 1.0f;
  s_reset_tick_base += 10000U;
  s_tick = s_reset_tick_base;
  s_save_pending = false;
  s_save_generation = 0U;
  s_save_process_count = 0U;
  s_rollback_count = 0U;
  s_discard_count = 0U;
  s_vofa_result_count = 0U;
  s_vofa_terminal_failed_count = 0U;
  s_error_report_count = 0U;
}

static int TestTerminalFailureRollsBackAndReleasesLease(void) {
  ResetState();
  if (!CmdService_BeginScheduledSave()) {
    return 1;
  }
  CmdService_CommitScheduledSave();

  CmdService_Process();
  s_tick += 250U;
  CmdService_Process();
  s_tick += 250U;
  CmdService_Process();

  if (s_save_process_count != 3U || s_discard_count != 1U ||
      s_rollback_count != 1U) {
    return 1;
  }
  if (s_save_pending || g_ds402_state_machine.maintenance_active) {
    return 1;
  }
  return s_vofa_result_count != 3U || s_vofa_terminal_failed_count != 1U ||
         s_error_report_count != 0U;
}

static int TestDirectScheduledSaveTerminalFailureRollsBack(void) {
  ResetState();
  Param_ScheduleSave();

  CmdService_Process();
  s_tick += 250U;
  CmdService_Process();
  s_tick += 250U;
  CmdService_Process();

  if (s_save_process_count != 3U || s_discard_count != 1U ||
      s_rollback_count != 1U) {
    return 1;
  }
  if (s_save_pending || g_ds402_state_machine.maintenance_active) {
    return 1;
  }
  return s_vofa_result_count != 0U || s_vofa_terminal_failed_count != 0U ||
         s_error_report_count != 0U;
}

int main(void) {
  if (TestTerminalFailureRollsBackAndReleasesLease()) {
    printf("FAIL terminal scheduled-save failure did not rollback cleanly\n");
    return 1;
  }
  if (TestDirectScheduledSaveTerminalFailureRollsBack()) {
    printf("FAIL direct scheduled-save failure did not rollback cleanly\n");
    return 1;
  }
  printf("Command service persistent rollback tests passed\n");
  return 0;
}
