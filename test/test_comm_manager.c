#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "calibration_context.h"
#include "device_id.h"
#include "error_types.h"
#include "executor.h"
#include "manager.h"
#include "motor.h"
#include "protocol/canopen/canopen_protocol.h"
#include "protocol/mit/mit_protocol.h"
#include "protocol/vector/vector_protocol.h"

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      return 1;                                                                \
    }                                                                          \
  } while (0)

uint8_t g_can_id = 1U;
uint8_t g_protocol_type = 0U;
MOTOR_DATA motor_data;
static ParseResult s_parse_result;
static unsigned s_execute_count;
static unsigned s_watchdog_feed_count;
static uint32_t s_watchdog_timestamp;
static unsigned s_parse_call_count;
static unsigned s_send_count;
static unsigned s_build_feedback_count;
static unsigned s_error_report_count;
static uint32_t s_last_error_code;
static bool s_request_feedback;
static TransportRxCallback s_registered_rx_callback;
static unsigned s_register_rx_callback_count;
static bool s_register_rx_callback_should_succeed;

static void reset_fixture(void) {
  s_register_rx_callback_should_succeed = true;
  (void)Protocol_RegisterTransport(NULL);
  s_parse_result = PARSE_OK;
  s_execute_count = 0U;
  s_watchdog_feed_count = 0U;
  s_watchdog_timestamp = 0U;
  s_parse_call_count = 0U;
  s_send_count = 0U;
  s_build_feedback_count = 0U;
  s_error_report_count = 0U;
  s_last_error_code = 0U;
  s_request_feedback = false;
  s_registered_rx_callback = NULL;
  s_register_rx_callback_count = 0U;
  memset(&motor_data, 0, sizeof(motor_data));
  Protocol_Init(PROTOCOL_VECTOR);
  Protocol_ResetStats();
}

static bool fake_send(const TransportFrame *frame) {
  (void)frame;
  s_send_count++;
  return true;
}

static bool fake_register_rx_callback(TransportRxCallback cb) {
  s_register_rx_callback_count++;
  if (!s_register_rx_callback_should_succeed) {
    return false;
  }
  s_registered_rx_callback = cb;
  return true;
}

static const TransportInterface s_valid_can_transport = {
    .send = fake_send,
    .register_rx_callback = fake_register_rx_callback,
    .type = TRANSPORT_CAN,
};

static bool register_valid_can_transport(void) {
  return Protocol_RegisterTransport(&s_valid_can_transport);
}

static int test_only_valid_frames_feed_can_watchdog(void) {
  reset_fixture();
  CAN_Frame frame = {0};
  frame.id = (1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  Protocol_ProcessRxFrame(&frame);
  CHECK(s_execute_count == 1U);
  CHECK(s_watchdog_feed_count == 1U);
  CHECK(s_watchdog_timestamp == 0U);

  s_parse_result = PARSE_ERR_INVALID_FRAME;
  Protocol_ProcessRxFrame(&frame);
  CHECK(s_watchdog_feed_count == 1U);

  s_parse_result = PARSE_UNKNOWN_ID;
  Protocol_ProcessRxFrame(&frame);
  CHECK(s_execute_count == 1U);
  CHECK(s_watchdog_feed_count == 1U);
  return 0;
}

static int test_rtr_frames_are_rejected_before_protocol_parse(void) {
  reset_fixture();
  CAN_Frame frame = {0};
  frame.id = (1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;
  frame.is_rtr = true;
  MotorCommand cmd = {0};

  CHECK(Protocol_ParseFrame(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
  CHECK(s_parse_call_count == 0U);
  CHECK(s_error_report_count == 1U);
  return 0;
}

static int test_mit_get_state_sends_feedback_without_executing_command(void) {
  reset_fixture();
  CHECK(register_valid_can_transport());
  Protocol_Init(PROTOCOL_MIT);
  s_request_feedback = true;
  CAN_Frame frame = {0};
  frame.id = g_can_id;
  frame.dlc = 2U;
  frame.data[0] = 0xFFU;
  frame.data[1] = 0x04U;

  Protocol_ProcessRxFrame(&frame);
  CHECK(s_parse_call_count == 1U);
  CHECK(s_execute_count == 0U);
  CHECK(s_build_feedback_count == 1U);
  CHECK(s_send_count == 1U);
  CHECK(s_watchdog_feed_count == 1U);
  return 0;
}

static int test_vector_get_id_fast_path_accepts_only_valid_query_frames(void) {
  reset_fixture();
  CHECK(register_valid_can_transport());
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)VECTOR_CMD_GET_ID << 24) | g_can_id;
  frame.dlc = 0U;
  frame.is_extended = true;

  Protocol_ProcessRxFrame(&frame);
  CHECK(s_parse_call_count == 0U);
  CHECK(s_execute_count == 0U);
  CHECK(s_send_count == 1U);
  CHECK(s_watchdog_feed_count == 1U);

  frame.id = ((uint32_t)VECTOR_CMD_GET_ID << 24) | 0x7FU;
  Protocol_ProcessRxFrame(&frame);
  CHECK(s_parse_call_count == 0U);
  CHECK(s_send_count == 2U);
  CHECK(s_watchdog_feed_count == 2U);

  s_parse_result = PARSE_ERR_INVALID_FRAME;
  frame.id = ((uint32_t)VECTOR_CMD_GET_ID << 24) | g_can_id;
  frame.is_extended = false;
  Protocol_ProcessRxFrame(&frame);
  CHECK(s_parse_call_count == 1U);
  CHECK(s_send_count == 2U);
  CHECK(s_watchdog_feed_count == 2U);

  frame.is_extended = true;
  frame.is_rtr = true;
  Protocol_ProcessRxFrame(&frame);
  CHECK(s_parse_call_count == 1U);
  CHECK(s_send_count == 2U);
  CHECK(s_watchdog_feed_count == 2U);

  s_parse_result = PARSE_UNKNOWN_ID;
  frame.is_rtr = false;
  frame.id = ((uint32_t)VECTOR_CMD_GET_ID << 24) | (g_can_id + 1U);
  Protocol_ProcessRxFrame(&frame);
  CHECK(s_parse_call_count == 2U);
  CHECK(s_send_count == 2U);
  CHECK(s_watchdog_feed_count == 2U);
  return 0;
}

static int test_queue_overflow_events_are_reported_once_per_new_drop(void) {
  reset_fixture();
  CAN_Frame frame = {0};
  frame.id = (1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  for (unsigned i = 0U; i < 40U; ++i) {
    (void)Protocol_QueueRxFrame(&frame);
  }
  Protocol_ProcessQueuedFrames();
  CommStats_t stats = {0};
  Protocol_GetStats(&stats);
  CHECK(stats.rx_overflow_events == 1U);
  CHECK(s_error_report_count == 1U);
  CHECK(s_last_error_code == ERROR_COMM_RX_QUEUE_OVERFLOW);

  for (unsigned i = 0U; i < 40U; ++i) {
    (void)Protocol_QueueRxFrame(&frame);
  }
  Protocol_ProcessQueuedFrames();
  Protocol_GetStats(&stats);
  CHECK(stats.rx_overflow_events == 2U);
  CHECK(s_error_report_count == 2U);
  CHECK(s_last_error_code == ERROR_COMM_RX_QUEUE_OVERFLOW);
  return 0;
}

static int test_register_transport_rejects_null_and_send_fails(void) {
  reset_fixture();
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  CHECK(!Protocol_RegisterTransport(NULL));
  CHECK(!Protocol_SendFrame(&frame));
  CHECK(s_send_count == 0U);
  return 0;
}

static int test_register_transport_rejects_missing_send_and_send_fails(void) {
  reset_fixture();
  const TransportInterface transport = {
      .register_rx_callback = fake_register_rx_callback,
      .type = TRANSPORT_CAN,
  };
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  CHECK(!Protocol_RegisterTransport(&transport));
  CHECK(s_register_rx_callback_count == 0U);
  CHECK(s_registered_rx_callback == NULL);
  CHECK(!Protocol_SendFrame(&frame));
  CHECK(s_send_count == 0U);
  return 0;
}

static int
test_register_transport_rejects_missing_rx_callback_and_send_fails(void) {
  reset_fixture();
  const TransportInterface transport = {
      .send = fake_send,
      .type = TRANSPORT_CAN,
  };
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  CHECK(!Protocol_RegisterTransport(&transport));
  CHECK(s_registered_rx_callback == NULL);
  CHECK(!Protocol_SendFrame(&frame));
  CHECK(s_send_count == 0U);
  return 0;
}

static int
test_register_transport_rejects_rx_callback_failure_and_send_fails(void) {
  reset_fixture();
  s_register_rx_callback_should_succeed = false;
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  CHECK(!Protocol_RegisterTransport(&s_valid_can_transport));
  CHECK(s_register_rx_callback_count == 1U);
  CHECK(s_registered_rx_callback == NULL);
  CHECK(!Protocol_SendFrame(&frame));
  CHECK(s_send_count == 0U);
  return 0;
}

static int
test_register_transport_accepts_valid_can_transport_and_send_succeeds(void) {
  reset_fixture();
  CAN_Frame frame = {0};
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.dlc = 8U;
  frame.is_extended = true;

  CHECK(register_valid_can_transport());
  CHECK(s_register_rx_callback_count == 1U);
  CHECK(s_registered_rx_callback != NULL);
  CHECK(Protocol_SendFrame(&frame));
  CHECK(s_send_count == 1U);
  return 0;
}

static int
test_registered_transport_queues_valid_can_frame_for_task_context(void) {
  reset_fixture();
  CHECK(register_valid_can_transport());
  CHECK(s_register_rx_callback_count == 1U);
  CHECK(s_registered_rx_callback != NULL);

  TransportFrame frame = {0};
  frame.type = TRANSPORT_CAN;
  frame.id = g_can_id;
  frame.len = 8U;
  frame.is_extended = false;
  frame.data[0] = 0x11U;
  frame.data[7] = 0x88U;

  s_registered_rx_callback(&frame);
  CHECK(s_execute_count == 0U);
  CHECK(s_parse_call_count == 0U);

  CommStats_t stats = {0};
  Protocol_GetStats(&stats);
  CHECK(stats.rx_frames_total == 1U);
  CHECK(stats.rx_queue_depth == 1U);
  CHECK(stats.rx_queue_peak == 1U);
  CHECK(stats.rx_frames_dropped == 0U);

  Protocol_ProcessQueuedFrames();
  CHECK(s_parse_call_count == 1U);
  CHECK(s_execute_count == 1U);
  CHECK(s_watchdog_feed_count == 1U);
  Protocol_GetStats(&stats);
  CHECK(stats.rx_queue_depth == 0U);
  return 0;
}

static int test_registered_transport_ignores_non_can_frame(void) {
  reset_fixture();
  CHECK(register_valid_can_transport());
  CHECK(s_registered_rx_callback != NULL);

  TransportFrame frame = {0};
  frame.type = TRANSPORT_USB;
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.len = 8U;
  frame.is_extended = true;

  s_registered_rx_callback(&frame);
  Protocol_ProcessQueuedFrames();

  CommStats_t stats = {0};
  Protocol_GetStats(&stats);
  CHECK(stats.rx_frames_total == 0U);
  CHECK(stats.rx_queue_depth == 0U);
  CHECK(s_parse_call_count == 0U);
  CHECK(s_execute_count == 0U);
  return 0;
}

static int test_registered_transport_ignores_oversized_can_frame(void) {
  reset_fixture();
  CHECK(register_valid_can_transport());
  CHECK(s_registered_rx_callback != NULL);

  TransportFrame frame = {0};
  frame.type = TRANSPORT_CAN;
  frame.id = ((uint32_t)1U << 24) | g_can_id;
  frame.len = 9U;
  frame.is_extended = true;

  s_registered_rx_callback(&frame);
  Protocol_ProcessQueuedFrames();

  CommStats_t stats = {0};
  Protocol_GetStats(&stats);
  CHECK(stats.rx_frames_total == 0U);
  CHECK(stats.rx_queue_depth == 0U);
  CHECK(s_parse_call_count == 0U);
  CHECK(s_execute_count == 0U);
  return 0;
}

int main(void) {
  int failures = test_only_valid_frames_feed_can_watchdog();
  failures += test_rtr_frames_are_rejected_before_protocol_parse();
  failures += test_mit_get_state_sends_feedback_without_executing_command();
  failures += test_vector_get_id_fast_path_accepts_only_valid_query_frames();
  failures += test_queue_overflow_events_are_reported_once_per_new_drop();
  failures += test_register_transport_rejects_null_and_send_fails();
  failures += test_register_transport_rejects_missing_send_and_send_fails();
  failures +=
      test_register_transport_rejects_missing_rx_callback_and_send_fails();
  failures +=
      test_register_transport_rejects_rx_callback_failure_and_send_fails();
  failures +=
      test_register_transport_accepts_valid_can_transport_and_send_succeeds();
  failures +=
      test_registered_transport_queues_valid_can_frame_for_task_context();
  failures += test_registered_transport_ignores_non_can_frame();
  failures += test_registered_transport_ignores_oversized_can_frame();
  if (failures == 0) {
    printf("All communication manager tests PASSED\n");
  }
  return failures;
}

void Executor_ProcessCommand(const MotorCommand *cmd) {
  (void)cmd;
  s_execute_count++;
}
void Detection_FeedWatchdog(uint32_t timestamp) {
  s_watchdog_feed_count++;
  s_watchdog_timestamp = timestamp;
}
uint32_t HAL_GetSystemTick(void) { return 0U; }
void ErrorManager_Report(uint32_t error_code, const char *message) {
  (void)message;
  s_error_report_count++;
  s_last_error_code = error_code;
}
void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line) {
  (void)error_code;
  (void)message;
  (void)file;
  (void)line;
  s_error_report_count++;
}
uint32_t Safety_GetActiveFaultBits(void) { return 0U; }
uint8_t CalibContext_GetProgress(uint8_t a, uint8_t b,
                                 const CalibrationContext *c) {
  (void)a;
  (void)b;
  (void)c;
  return 0U;
}

void ProtocolVector_Init(void) {}
void ProtocolVector_Service(void) {}
ParseResult ProtocolVector_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  (void)frame;
  s_parse_call_count++;
  memset(cmd, 0, sizeof(*cmd));
  return s_parse_result;
}
bool ProtocolVector_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}
bool ProtocolVector_BuildFault(uint32_t fault_code, uint32_t warning_code,
                               CAN_Frame *frame) {
  (void)fault_code;
  (void)warning_code;
  (void)frame;
  return false;
}
bool ProtocolVector_BuildParamResponse(uint16_t param_index, float value,
                                       CAN_Frame *frame) {
  (void)param_index;
  (void)value;
  (void)frame;
  return false;
}
bool ProtocolVector_BuildCalibStatus(const MotorStatus *status,
                                     CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}

void ProtocolCANopen_Init(void) {}
ParseResult ProtocolCANopen_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  (void)frame;
  (void)cmd;
  s_parse_call_count++;
  return PARSE_UNKNOWN_ID;
}
bool ProtocolCANopen_BuildFeedback(const MotorStatus *status,
                                   CAN_Frame *frame) {
  (void)status;
  (void)frame;
  return false;
}
bool ProtocolCANopen_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  (void)fault_code;
  (void)frame;
  return false;
}
bool ProtocolCANopen_BuildHeartbeat(uint32_t now_ms, CAN_Frame *frame) {
  (void)now_ms;
  (void)frame;
  return false;
}

void ProtocolMIT_Init(void) {}
ParseResult ProtocolMIT_Parse(const CAN_Frame *frame, MotorCommand *cmd) {
  (void)frame;
  s_parse_call_count++;
  memset(cmd, 0, sizeof(*cmd));
  if (s_request_feedback) {
    cmd->request_feedback = true;
    return PARSE_OK;
  }
  return PARSE_UNKNOWN_ID;
}
bool ProtocolMIT_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  (void)status;
  s_build_feedback_count++;
  if (frame != NULL) {
    memset(frame, 0, sizeof(*frame));
    frame->dlc = 6U;
  }
  return true;
}
bool ProtocolMIT_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  (void)fault_code;
  (void)frame;
  return false;
}
