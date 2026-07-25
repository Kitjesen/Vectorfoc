#include "app_comm_bootstrap.h"
#include "bsp_can.h"
#include "can_transport.h"
#include "manager.h"
#include "motor.h"
#include "safety_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);              \
      return 1;                                                                \
    }                                                                          \
  } while (0)

typedef enum {
  EVENT_TRANSPORT_INIT,
  EVENT_TRANSPORT_GET,
  EVENT_PROTOCOL_REGISTER,
  EVENT_PROTOCOL_INIT,
  EVENT_SAFETY_REGISTER,
  EVENT_BSP_CAN_INIT,
  EVENT_FAULT_REPORT,
} TestEvent;

static TestEvent s_events[10];
static uint32_t s_event_count;
static TransportInterface s_transport = {.test_tag = 0x1357u};
static const TransportInterface *s_registered_transport;
static ProtocolType s_initialized_protocol;
static SafetyFaultCallback s_fault_callback;
static uint32_t s_reported_fault_bits;
static MOTOR_DATA *s_reported_motor;
static bool s_protocol_report_result;
static bool s_transport_init_result;
static bool s_protocol_register_result;
static bool s_bsp_can_init_results[2];
static uint32_t s_bsp_can_init_count;
static BSP_CAN_BaudrateId s_bsp_can_baudrates[2];

uint8_t g_protocol_type;
uint8_t g_can_baudrate;

static void Record(TestEvent event) {
  if (s_event_count < sizeof(s_events) / sizeof(s_events[0])) {
    s_events[s_event_count++] = event;
  }
}

bool BSP_CAN_Init(BSP_CAN_BaudrateId baudrate_id) {
  Record(EVENT_BSP_CAN_INIT);
  if (s_bsp_can_init_count < 2u) {
    s_bsp_can_baudrates[s_bsp_can_init_count] = baudrate_id;
  }
  const uint32_t index = s_bsp_can_init_count++;
  if (index < 2u) {
    return s_bsp_can_init_results[index];
  }
  return false;
}

bool CAN_Transport_Init(void) {
  Record(EVENT_TRANSPORT_INIT);
  return s_transport_init_result;
}

const TransportInterface *CAN_Transport_GetInterface(void) {
  Record(EVENT_TRANSPORT_GET);
  return &s_transport;
}

bool Protocol_RegisterTransport(const TransportInterface *transport) {
  Record(EVENT_PROTOCOL_REGISTER);
  s_registered_transport = transport;
  return s_protocol_register_result;
}

void Protocol_Init(ProtocolType default_protocol) {
  Record(EVENT_PROTOCOL_INIT);
  s_initialized_protocol = default_protocol;
}

bool Protocol_ReportFaultCallback(uint32_t fault_bits, MOTOR_DATA *motor) {
  Record(EVENT_FAULT_REPORT);
  s_reported_fault_bits = fault_bits;
  s_reported_motor = motor;
  return s_protocol_report_result;
}

void Safety_RegisterFaultCallback(SafetyFaultCallback callback) {
  Record(EVENT_SAFETY_REGISTER);
  s_fault_callback = callback;
}

static void ResetState(void) {
  s_event_count = 0u;
  s_registered_transport = NULL;
  s_initialized_protocol = PROTOCOL_VECTOR;
  s_fault_callback = NULL;
  s_reported_fault_bits = 0u;
  s_reported_motor = NULL;
  s_protocol_report_result = true;
  s_transport_init_result = true;
  s_protocol_register_result = true;
  s_bsp_can_init_results[0] = true;
  s_bsp_can_init_results[1] = true;
  s_bsp_can_init_count = 0u;
  s_bsp_can_baudrates[0] = BSP_CAN_BAUD_1M;
  s_bsp_can_baudrates[1] = BSP_CAN_BAUD_1M;
  g_protocol_type = PROTOCOL_VECTOR;
  g_can_baudrate = BSP_CAN_BAUD_1M;
}

static int CheckBootstrapOrderWithBspAttempts(uint32_t bsp_attempts) {
  static const TestEvent expected[] = {
      EVENT_TRANSPORT_INIT, EVENT_TRANSPORT_GET,   EVENT_PROTOCOL_REGISTER,
      EVENT_PROTOCOL_INIT,  EVENT_SAFETY_REGISTER, EVENT_BSP_CAN_INIT,
      EVENT_BSP_CAN_INIT,
  };
  const uint32_t expected_count = 5u + bsp_attempts;

  CHECK(bsp_attempts <= 2u);
  CHECK(s_event_count == expected_count);
  for (uint32_t i = 0u; i < s_event_count; ++i) {
    CHECK(s_events[i] == expected[i]);
  }
  CHECK(s_registered_transport == &s_transport);
  CHECK(s_fault_callback != NULL);
  return 0;
}

static int test_returns_true_when_all_bootstrap_steps_succeed(void) {
  ResetState();
  g_protocol_type = PROTOCOL_MIT;
  g_can_baudrate = BSP_CAN_BAUD_500K;

  CHECK(AppComm_Bootstrap());

  CHECK(CheckBootstrapOrderWithBspAttempts(1u) == 0);
  CHECK(s_initialized_protocol == PROTOCOL_MIT);
  CHECK(s_bsp_can_init_count == 1u);
  CHECK(s_bsp_can_baudrates[0] == BSP_CAN_BAUD_500K);
  return 0;
}

static int
test_registered_safety_callback_reports_faults_through_protocol(void) {
  MOTOR_DATA motor = {.test_tag = 0x2468u};

  ResetState();
  CHECK(AppComm_Bootstrap());

  CHECK(s_fault_callback(0x5Au, &motor));
  CHECK(s_reported_fault_bits == 0x5Au);
  CHECK(s_reported_motor == &motor);
  s_protocol_report_result = false;
  CHECK(!s_fault_callback(0xA5u, &motor));
  CHECK(s_reported_fault_bits == 0xA5u);
  CHECK(s_reported_motor == &motor);
  return 0;
}

static int test_normalizes_invalid_saved_baudrate_before_init(void) {
  ResetState();
  g_can_baudrate = 99u;

  CHECK(AppComm_Bootstrap());

  CHECK(CheckBootstrapOrderWithBspAttempts(1u) == 0);
  CHECK(s_bsp_can_init_count == 1u);
  CHECK(s_bsp_can_baudrates[0] == BSP_CAN_BAUD_1M);
  CHECK(g_can_baudrate == BSP_CAN_BAUD_1M);
  return 0;
}
static int test_returns_false_when_transport_init_fails(void) {
  ResetState();
  s_transport_init_result = false;

  CHECK(!AppComm_Bootstrap());

  CHECK(s_event_count == 1u);
  CHECK(s_events[0] == EVENT_TRANSPORT_INIT);
  CHECK(s_bsp_can_init_count == 0u);
  CHECK(s_registered_transport == NULL);
  CHECK(s_fault_callback == NULL);
  return 0;
}

static int test_returns_false_when_protocol_transport_registration_fails(void) {
  ResetState();
  s_protocol_register_result = false;

  CHECK(!AppComm_Bootstrap());

  CHECK(s_event_count == 3u);
  CHECK(s_events[0] == EVENT_TRANSPORT_INIT);
  CHECK(s_events[1] == EVENT_TRANSPORT_GET);
  CHECK(s_events[2] == EVENT_PROTOCOL_REGISTER);
  CHECK(s_bsp_can_init_count == 0u);
  CHECK(s_fault_callback == NULL);
  return 0;
}
static int test_does_not_retry_valid_baud_when_hardware_init_fails(void) {
  ResetState();
  g_can_baudrate = BSP_CAN_BAUD_250K;
  s_bsp_can_init_results[0] = false;

  CHECK(!AppComm_Bootstrap());

  CHECK(CheckBootstrapOrderWithBspAttempts(1u) == 0);
  CHECK(s_bsp_can_init_count == 1u);
  CHECK(s_bsp_can_baudrates[0] == BSP_CAN_BAUD_250K);
  CHECK(g_can_baudrate == BSP_CAN_BAUD_250K);
  return 0;
}
static int test_defaults_invalid_protocol_to_vector(void) {
  ResetState();
  g_protocol_type = 99u;

  CHECK(AppComm_Bootstrap());

  CHECK(CheckBootstrapOrderWithBspAttempts(1u) == 0);
  CHECK(s_initialized_protocol == PROTOCOL_VECTOR);
  CHECK(g_protocol_type == PROTOCOL_VECTOR);
  return 0;
}

int main(void) {
  CHECK(test_returns_true_when_all_bootstrap_steps_succeed() == 0);
  CHECK(test_registered_safety_callback_reports_faults_through_protocol() == 0);
  CHECK(test_normalizes_invalid_saved_baudrate_before_init() == 0);
  CHECK(test_returns_false_when_transport_init_fails() == 0);
  CHECK(test_returns_false_when_protocol_transport_registration_fails() == 0);
  CHECK(test_does_not_retry_valid_baud_when_hardware_init_fails() == 0);
  CHECK(test_defaults_invalid_protocol_to_vector() == 0);

  puts("App communication bootstrap tests PASSED");
  return 0;
}
