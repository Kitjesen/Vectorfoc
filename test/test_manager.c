#include "manager.h"
#include "fsm.h"
#include "inovxio_protocol.h"
#include "motor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int tests_passed = 0;

#define TEST(name)                                                            \
  do {                                                                        \
    reset_test_state();                                                       \
    printf("  Testing: %s ... ", #name);                                      \
    test_##name();                                                            \
    printf("PASS\n");                                                         \
    tests_passed++;                                                           \
  } while (0)

uint8_t g_can_id = 5;
uint8_t g_can_baudrate = 0;
uint8_t g_protocol_type = 0;
uint32_t g_can_timeout_ms = 0;
uint8_t g_run_mode = 0;
MOTOR_DATA motor_data = {0};
StateMachine g_ds402_state_machine = {0};

static CAN_Frame s_last_sent_frame = {0};
static int s_send_count = 0;
static int s_private_init_count = 0;
static int s_private_parse_count = 0;
static int s_executor_call_count = 0;
static int s_observer_call_count = 0;
static int s_error_report_count = 0;
static CAN_Frame s_observed_frame = {0};
static MotorCommand s_next_cmd = {0};
static ParseResult s_parse_result = PARSE_UNKNOWN_ID;

static void reset_test_state(void) {
  memset(&s_last_sent_frame, 0, sizeof(s_last_sent_frame));
  memset(&s_observed_frame, 0, sizeof(s_observed_frame));
  memset(&s_next_cmd, 0, sizeof(s_next_cmd));
  s_send_count = 0;
  s_private_init_count = 0;
  s_private_parse_count = 0;
  s_executor_call_count = 0;
  s_observer_call_count = 0;
  s_error_report_count = 0;
  s_parse_result = PARSE_UNKNOWN_ID;
  g_can_id = 5;
  Protocol_Init(PROTOCOL_INOVXIO);
}

static void manager_observer(const CAN_Frame *frame) {
  s_observer_call_count++;
  s_observed_frame = *frame;
}

void ProtocolPrivate_Init(void) { s_private_init_count++; }

void ProtocolCANopen_Init(void) {}

void ProtocolMIT_Init(void) {}

ParseResult ProtocolPrivate_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  (void)frame;
  s_private_parse_count++;
  *cmd = s_next_cmd;
  return s_parse_result;
}

ParseResult ProtocolCANopen_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  (void)frame;
  (void)cmd;
  return PARSE_UNKNOWN_ID;
}

ParseResult ProtocolMIT_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  (void)frame;
  (void)cmd;
  return PARSE_UNKNOWN_ID;
}

bool ProtocolPrivate_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}

bool ProtocolCANopen_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}

bool ProtocolMIT_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}

bool ProtocolPrivate_BuildFault(uint32_t fault_code, uint32_t warning_code,
                                CAN_Frame *frame) {
  (void)fault_code;
  (void)warning_code;
  (void)frame;
  return false;
}

bool ProtocolCANopen_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  (void)fault_code;
  (void)frame;
  return false;
}

bool ProtocolMIT_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  (void)fault_code;
  (void)frame;
  return false;
}

bool ProtocolPrivate_BuildParamResponse(uint16_t param_index, float value,
                                        CAN_Frame *frame) {
  (void)param_index;
  (void)value;
  (void)frame;
  return false;
}

bool ProtocolCANopen_BuildHeartbeat(uint32_t now_ms, CAN_Frame *frame) {
  (void)now_ms;
  (void)frame;
  return false;
}

void Executor_ProcessCommand(const MotorCommand *cmd) {
  s_executor_call_count++;
  s_next_cmd = *cmd;
}

bool BSP_CAN_SendFrame(const CAN_Frame *frame) {
  s_last_sent_frame = *frame;
  s_send_count++;
  return true;
}

void ErrorManager_Report(uint32_t error_code, const char *message) {
  (void)error_code;
  (void)message;
  s_error_report_count++;
}

void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line) {
  (void)error_code;
  (void)message;
  (void)file;
  (void)line;
  s_error_report_count++;
}

uint32_t Safety_GetActiveFaultBits(void) { return 0; }

uint32_t Safety_GetLastFaultTime(void) { return 0; }

void Safety_ClearFaults(StateMachine *fsm) { (void)fsm; }

static void test_get_id_broadcast_sends_uid_response_and_observer_runs(void) {
  CAN_Frame frame = {0};

  Protocol_RegisterRxObserver(manager_observer);

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | 0x7FU;
  frame.is_extended = true;
  frame.dlc = 0;

  Protocol_ProcessRxFrame(&frame);

  assert(s_private_init_count == 1);
  assert(s_private_parse_count == 0);
  assert(s_observer_call_count == 1);
  assert(s_observed_frame.id == frame.id);
  assert(s_send_count == 1);
  assert(s_last_sent_frame.id ==
         (((uint32_t)PRIVATE_CMD_GET_ID << 24) |
          ((uint32_t)g_can_id << 8) | 0xFEU));
  assert(s_last_sent_frame.dlc == 8);
  assert(s_last_sent_frame.data[0] == 0x44U);
  assert(s_last_sent_frame.data[1] == 0x33U);
  assert(s_last_sent_frame.data[2] == 0x22U);
  assert(s_last_sent_frame.data[3] == 0x11U);
}

static void test_get_id_target_match_sends_uid_response_without_parse(void) {
  CAN_Frame frame = {0};

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | g_can_id;
  frame.is_extended = true;
  frame.dlc = 0;

  Protocol_ProcessRxFrame(&frame);

  assert(s_private_parse_count == 0);
  assert(s_send_count == 1);
  assert(s_executor_call_count == 0);
}

static void test_get_id_other_target_is_ignored(void) {
  CAN_Frame frame = {0};

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | 0x09U;
  frame.is_extended = true;
  frame.dlc = 0;

  Protocol_ProcessRxFrame(&frame);

  assert(s_private_parse_count == 0);
  assert(s_send_count == 0);
  assert(s_executor_call_count == 0);
  assert(s_error_report_count == 0);
}

static void test_parse_ok_path_executes_command(void) {
  CAN_Frame frame = {0};

  s_parse_result = PARSE_OK;
  s_next_cmd.has_enable_command = true;
  s_next_cmd.enable_motor = true;

  frame.id = ((uint32_t)PRIVATE_CMD_MOTOR_ENABLE << 24) | g_can_id;
  frame.is_extended = true;
  frame.dlc = 0;

  Protocol_ProcessRxFrame(&frame);

  assert(s_private_parse_count == 1);
  assert(s_executor_call_count == 1);
  assert(s_next_cmd.enable_motor);
}

static void test_non_target_private_command_is_ignored(void) {
  CAN_Frame frame = {0};

  s_parse_result = PARSE_OK;
  s_next_cmd.has_enable_command = true;
  s_next_cmd.enable_motor = true;

  frame.id = ((uint32_t)PRIVATE_CMD_MOTOR_ENABLE << 24) | 0x09U;
  frame.is_extended = true;
  frame.dlc = 0;

  Protocol_ProcessRxFrame(&frame);

  assert(s_private_parse_count == 0);
  assert(s_executor_call_count == 0);
  assert(s_error_report_count == 0);
}

int main(void) {
  printf("Running manager tests...\n");

  TEST(get_id_broadcast_sends_uid_response_and_observer_runs);
  TEST(get_id_target_match_sends_uid_response_without_parse);
  TEST(get_id_other_target_is_ignored);
  TEST(parse_ok_path_executes_command);
  TEST(non_target_private_command_is_ignored);

  printf("All %d manager tests passed.\n", tests_passed);
  return 0;
}
