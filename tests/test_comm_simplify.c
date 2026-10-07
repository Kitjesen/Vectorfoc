// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/* Real manager, three codecs and communication task; only hardware, command
 * execution, safety state and Flash are substituted at their boundaries. */
#include "protocol_dispatcher.h"
#include "communication_task.h"
#include "motor_runtime.h"
#include "parameter_access.h"
#include "calibration_state.h"
#include "cmsis_os.h"
#include "error_manager.h"
#include "safety_manager.h"
#include "protocol_vector.h"
#include "drive_state_machine.h"
#include "bootloader.h"
#include "watchdog_supervisor.h"
#include <stdio.h>
#include <string.h>

#define CHECK(expr) do { \
  if (!(expr)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    return 1; \
  } \
} while (0)

uint8_t g_can_id = 1;
uint8_t g_protocol_type = 0;
MOTOR_DATA motor_data;
StateMachine g_ds402_state_machine;
static CAN_Frame sent[64];
static unsigned sent_count;
static bool send_ok = true;
static MotorCommand executed[64];
static unsigned executed_count;
static uint32_t now_ms;
static uint32_t faults;
static bool save_pending;
static unsigned saves;
static unsigned save_service_calls;
static unsigned errors;
static unsigned calibration_requests;
static uint32_t save_generation;

bool BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t callback) {
  return callback != NULL;
}
bool BSP_CAN_SendFrame(const BSP_CAN_Frame *frame) {
  if (sent_count < 64) {
    sent[sent_count].id = frame->id;
    sent[sent_count].dlc = frame->dlc;
    sent[sent_count].is_extended = frame->is_extended;
    memcpy(sent[sent_count].data, frame->data, sizeof(frame->data));
  }
  ++sent_count;
  return send_ok;
}
bool BSP_CAN_SendTrackedFrame(const BSP_CAN_Frame *frame,
                              BSP_CAN_TxTicket *ticket) {
  bool accepted = BSP_CAN_SendFrame(frame);
  if (accepted && ticket != NULL) {
    ticket->marker = sent_count;
    ticket->tx_buffer_mask = 1u;
  }
  return accepted;
}
bool BSP_CAN_TxTicketIsComplete(const BSP_CAN_TxTicket *ticket) {
  return ticket != NULL && ticket->marker != 0u;
}
void BSP_CAN_CancelTrackedSend(const BSP_CAN_TxTicket *ticket) {
  (void)ticket;
}
void Executor_ProcessCommand(const MotorCommand *cmd) {
  if (executed_count < 64) executed[executed_count] = *cmd;
  ++executed_count;
}
uint32_t HAL_GetTick(void) { return now_ms; }
osStatus osDelay(uint32_t millisec) { (void)millisec; return osOK; }
uint32_t Safety_GetActiveFaultBits(void) { return faults; }
uint32_t Safety_GetLastFaultTime(void) { return 0; }
void Param_ScheduleSave(void) { save_pending = true; ++save_generation; }
bool Param_HasScheduledSave(void) { return save_pending; }
uint32_t Param_GetScheduledSaveGeneration(void) { return save_generation; }
bool Param_DiscardScheduledSaveIfGeneration(uint32_t generation) {
  if (!save_pending || generation != save_generation) return false;
  save_pending = false;
  return true;
}
ParamResult Param_RollbackScheduledSave(void) { return PARAM_OK; }
bool Param_ProcessScheduledSave(void) {
  ++save_service_calls;
  bool requested = save_pending;
  if (requested) { ++saves; save_pending = false; }
  return requested;
}
ParamResult Param_WriteFloat(uint16_t index, float value) {
  (void)index; (void)value; return PARAM_OK;
}
ParamResult Param_WriteUint8(uint16_t index, uint8_t value) {
  (void)index; (void)value; return PARAM_OK;
}
void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t type) {
  (void)motor; (void)type; ++calibration_requests;
}
void Motor_AbortCalibration(MOTOR_DATA *motor) { (void)motor; }
bool Motor_ClearFaults(MOTOR_DATA *motor) { (void)motor; return true; }
uint8_t Motor_PreCalibCheck(MOTOR_DATA *motor, uint8_t *fail_mask) {
  (void)motor; if (fail_mask) *fail_mask = 0; return 0xFF;
}
uint8_t CalibContext_GetProgress(uint8_t stage, uint8_t substage,
                               const CalibrationContext *context) {
  (void)stage; (void)substage; (void)context; return 37;
}
void ErrorManager_Report(uint32_t code, const char *message) {
  (void)code; (void)message; ++errors;
}
void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)file; (void)line; ErrorManager_Report(code, message);
}
void Detection_FeedWatchdog(uint32_t timestamp) { (void)timestamp; }
void Emergency_DisableBridgeOutputs(void) {}
void Emergency_Shutdown(void) {}
void Boot_RequestUpgrade(void) {}
bool StateMachine_RequestState(StateMachine *sm, MotorState target_state) {
  (void)sm; (void)target_state; return true;
}
bool StateMachine_BeginMaintenance(StateMachine *sm) { (void)sm; return true; }
void StateMachine_EndMaintenance(StateMachine *sm) { (void)sm; }
void Vofa_ReportScheduledSaveResult(bool succeeded) { (void)succeeded; }
void Vofa_ReportScheduledSaveFailed(void) {}
void WatchdogSupervisor_MarkComm(void) {}

static CAN_Frame vector_command(uint8_t command) {
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)command << 24) | g_can_id;
  frame.is_extended = true;
  return frame;
}

static int test_tx_normalization_and_statistics(void) {
  CAN_Frame frame = {0};
  frame.id = 0x1234567;
  frame.dlc = 255;
  frame.is_extended = true;
  frame.is_rtr = true;
  for (unsigned i = 0; i < 8; ++i) frame.data[i] = (uint8_t)(0xA0 + i);
  CAN_Frame original = frame;
  Protocol_ResetStats();
  sent_count = 0;
  CHECK(!Protocol_SendFrame(&frame));
  CHECK(sent_count == 0);
  frame.dlc = 8;
  CHECK(!Protocol_SendFrame(&frame));
  CHECK(sent_count == 0);
  frame.is_rtr = false;
  CHECK(Protocol_SendFrame(&frame));
  CHECK(sent_count == 1 && sent[0].id == frame.id);
  CHECK(sent[0].is_extended && !sent[0].is_rtr && sent[0].dlc == 8);
  CHECK(memcmp(sent[0].data, frame.data, 8) == 0);
  CHECK(original.dlc == 255 && original.is_rtr);
  frame.id = 0x321;
  frame.dlc = 2;
  frame.is_extended = false;
  CHECK(Protocol_SendFrame(&frame));
  CHECK(sent[1].id == 0x321 && sent[1].dlc == 2);
  CHECK(!sent[1].is_extended && !sent[1].is_rtr);
  CHECK(memcmp(sent[1].data, frame.data, 2) == 0);
  send_ok = false;
  CHECK(!Protocol_SendFrame(&frame));
  send_ok = true;
  CHECK(!Protocol_SendFrame(NULL) && sent_count == 3);
  CommStats_t stats;
  Protocol_GetStats(&stats);
  CHECK(stats.tx_frames_total == 2 && stats.tx_frames_failed == 1);
  return 0;
}

static int test_three_protocol_feedback_wire_bytes(void) {
  const ProtocolType protocols[] = {PROTOCOL_VECTOR, PROTOCOL_CANOPEN, PROTOCOL_MIT};
  const uint32_t ids[] = {0x028001FD, 0x181, 1};
  const uint8_t lengths[] = {8, 8, 6};
  const uint8_t payloads[][8] = {
      {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA},
      {0, 0, 0, 0, 0, 0, 0, 0},
      {0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x30, 0, 0}};
  MotorStatus status = {0};
  status.can_id = 1;
  status.temperature = 25;
  status.motor_state = 3;
  sent_count = 0;
  for (unsigned i = 0; i < 3; ++i) {
    CAN_Frame frame = {0};
    Protocol_Init(protocols[i]);
    CHECK(Protocol_BuildFeedback(&status, &frame));
    CHECK(Protocol_SendFrame(&frame));
    CHECK(sent[i].id == ids[i] && sent[i].dlc == lengths[i]);
    CHECK(sent[i].is_extended == (i == 0) && !sent[i].is_rtr);
    CHECK(memcmp(sent[i].data, payloads[i], lengths[i]) == 0);
  }
  return 0;
}

static int test_parameter_response_and_fifo(void) {
  Protocol_Init(PROTOCOL_VECTOR);
  CAN_Frame response = {0};
  const uint8_t expected[8] = {0x10, 0x20, 0, 0, 0, 0, 0xA0, 0x3F};
  sent_count = 0;
  CHECK(Protocol_BuildParamResponse(0x2010, 1.25f, &response));
  CHECK(Protocol_SendFrame(&response));
  CHECK(sent[0].id == 0x120001FD && sent[0].dlc == 8);
  CHECK(memcmp(sent[0].data, expected, 8) == 0);

  Protocol_ResetStats();
  executed_count = 0;
  errors = 0;
  for (unsigned i = 0; i < 31; ++i) {
    CAN_Frame request = vector_command(VECTOR_CMD_PARAM_READ);
    request.dlc = 8;
    request.data[0] = (uint8_t)i;
    request.data[1] = 0x20;
    CHECK(Protocol_QueueRxFrame(&request));
  }
  CAN_Frame overflow = vector_command(VECTOR_CMD_PARAM_READ);
  CHECK(!Protocol_QueueRxFrame(&overflow));
  CHECK(executed_count == 0); /* ISR enqueue must not execute commands. */
  Protocol_ProcessQueuedFrames();
  CHECK(executed_count == 31 && errors == 1);
  for (unsigned i = 0; i < 31; ++i) {
    CHECK(executed[i].is_param_read && !executed[i].is_param_write);
    CHECK(executed[i].param_index == 0x2000 + i);
  }
  CommStats_t stats;
  Protocol_GetStats(&stats);
  CHECK(stats.rx_frames_total == 31 && stats.rx_frames_dropped == 1);
  CHECK(stats.rx_overflow_events == 1 && stats.rx_queue_depth == 0);
  CHECK(stats.rx_queue_peak == 31);
  return 0;
}

static int test_task_reports_and_deferred_save(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  motor_data.parameters.Rs = 0.5f;
  motor_data.parameters.pole_pairs = 7;
  Protocol_Init(PROTOCOL_VECTOR);
  CommTask_SetReportEnabled(false);
  sent_count = 0;
  now_ms = 0;
  CommTask_Process();
  CHECK(sent_count == 0 && save_service_calls == 0);

  CAN_Frame report = vector_command(VECTOR_CMD_REPORT);
  report.dlc = 1;
  report.data[0] = 1;
  CAN_Frame save = vector_command(VECTOR_CMD_SAVE);
  CHECK(Protocol_QueueRxFrame(&report));
  CHECK(Protocol_QueueRxFrame(&save));
  CHECK(!save_pending && saves == 0 && sent_count == 0);
  now_ms = 10;
  CommTask_Process();
  CHECK(saves == 1 && !save_pending && save_service_calls == 1);
  CHECK(sent_count == 2 && (sent[0].id >> 24) == VECTOR_CMD_SAVE);
  CHECK((sent[1].id >> 24) == 2);
  now_ms = 19;
  CommTask_Process();
  CHECK(sent_count == 2 && saves == 1);
  now_ms = 20;
  CommTask_Process();
  CHECK(sent_count == 3);

  faults = FAULT_OVER_TEMP;
  now_ms = 30;
  CommTask_Process();
  CHECK(sent_count == 4 && (sent[3].id >> 24) == 0x15);
  now_ms = 40;
  CommTask_Process();
  CHECK(sent_count == 4); /* Fault latches suppress motor feedback. */
  report.data[0] = 0;
  CHECK(Protocol_QueueRxFrame(&report));
  faults = 0;
  now_ms = 50;
  CommTask_Process();
  CHECK(sent_count == 4); /* REPORT=0 applies before reporting this tick. */

  motor_data.state.Sub_State = CURRENT_CALIBRATING;
  motor_data.state.Cs_State = CS_STATE_IDLE;
  now_ms = 60;
  CommTask_Process();
  CHECK(sent_count == 5 && sent[4].id == 0x090001FD);
  const uint8_t calibration[8] = {CURRENT_CALIBRATING, CS_STATE_IDLE, 37, 0,
                                  0x01, 0xF4, 0x00, 0x07};
  CHECK(memcmp(sent[4].data, calibration, 8) == 0);
  now_ms = 1059;
  CommTask_Process();
  CHECK(sent_count == 5);
  now_ms = 1060;
  CommTask_Process();
  CHECK(sent_count == 6 && sent[5].id == 0x090001FD);
  motor_data.state.Sub_State = SUB_STATE_IDLE;
  now_ms = 1061;
  CommTask_Process();
  CHECK(sent_count == 7 && sent[6].data[0] == SUB_STATE_IDLE);

  Protocol_Init(PROTOCOL_CANOPEN);
  now_ms = 2000;
  CommTask_Process();
  CHECK(sent_count == 8 && sent[7].id == 0x701 && sent[7].dlc == 1);
  CHECK(sent[7].data[0] == 0x7F && !sent[7].is_extended);
  now_ms = 2999;
  CommTask_Process();
  CHECK(sent_count == 8);
  now_ms = 3000;
  CommTask_Process();
  CHECK(sent_count == 9 && sent[8].id == 0x701);
  return 0;
}

static int test_malformed_calibration_never_reaches_motor(void) {
  Protocol_Init(PROTOCOL_VECTOR);
  MotorCommand command = {0};
  CAN_Frame frame = vector_command(VECTOR_CMD_CALIBRATE);
  calibration_requests = 0;
  frame.dlc = 0;
  CHECK(ProtocolVector_Parse(&frame, &command) == PARSE_ERR_INVALID_FRAME);
  frame.dlc = 1;
  frame.data[0] = 0;
  CHECK(ProtocolVector_Parse(&frame, &command) == PARSE_ERR_INVALID_FRAME);
  frame.data[0] = 255;
  CHECK(ProtocolVector_Parse(&frame, &command) == PARSE_ERR_INVALID_FRAME);
  CHECK(calibration_requests == 0);
  frame.data[0] = 3;
  CHECK(ProtocolVector_Parse(&frame, &command) == PARSE_OK);
  CHECK(calibration_requests == 1);
  return 0;
}
int main(void) {
  int failed = 0;
  failed += test_tx_normalization_and_statistics();
  failed += test_three_protocol_feedback_wire_bytes();
  failed += test_parameter_response_and_fifo();
  failed += test_task_reports_and_deferred_save();
  failed += test_malformed_calibration_never_reaches_motor();
  if (failed) return 1;
  puts("Communication simplification: 5 regression groups passed.");
  return 0;
}
