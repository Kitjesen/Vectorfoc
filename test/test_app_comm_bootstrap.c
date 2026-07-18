#include "app_comm_bootstrap.h"
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
  EVENT_BSP_CAN_INIT,
  EVENT_TRANSPORT_INIT,
  EVENT_TRANSPORT_GET,
  EVENT_PROTOCOL_REGISTER,
  EVENT_PROTOCOL_INIT,
  EVENT_SAFETY_REGISTER,
  EVENT_FAULT_REPORT,
} TestEvent;

static TestEvent s_events[8];
static uint32_t s_event_count;
static TransportInterface s_transport = {.test_tag = 0x1357u};
static const TransportInterface *s_registered_transport;
static ProtocolType s_initialized_protocol;
static SafetyFaultCallback s_fault_callback;
static uint32_t s_reported_fault_bits;
static MOTOR_DATA *s_reported_motor;
static bool s_protocol_report_result;

uint8_t g_protocol_type;

static void Record(TestEvent event) {
  if (s_event_count < sizeof(s_events) / sizeof(s_events[0])) {
    s_events[s_event_count++] = event;
  }
}

void BSP_CAN_Init(void) { Record(EVENT_BSP_CAN_INIT); }

void CAN_Transport_Init(void) { Record(EVENT_TRANSPORT_INIT); }

const TransportInterface *CAN_Transport_GetInterface(void) {
  Record(EVENT_TRANSPORT_GET);
  return &s_transport;
}

void Protocol_RegisterTransport(const TransportInterface *transport) {
  Record(EVENT_PROTOCOL_REGISTER);
  s_registered_transport = transport;
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
}

static int CheckBootstrapOrder(void) {
  static const TestEvent expected[] = {
      EVENT_BSP_CAN_INIT,      EVENT_TRANSPORT_INIT, EVENT_TRANSPORT_GET,
      EVENT_PROTOCOL_REGISTER, EVENT_PROTOCOL_INIT,  EVENT_SAFETY_REGISTER,
  };

  CHECK(s_event_count == sizeof(expected) / sizeof(expected[0]));
  for (uint32_t i = 0u; i < s_event_count; ++i) {
    CHECK(s_events[i] == expected[i]);
  }
  CHECK(s_registered_transport == &s_transport);
  CHECK(s_fault_callback != NULL);
  return 0;
}

int main(void) {
  MOTOR_DATA motor = {.test_tag = 0x2468u};

  ResetState();
  g_protocol_type = PROTOCOL_MIT;
  AppComm_Bootstrap();
  CHECK(CheckBootstrapOrder() == 0);
  CHECK(s_initialized_protocol == PROTOCOL_MIT);
  CHECK(s_fault_callback(0x5Au, &motor));
  CHECK(s_reported_fault_bits == 0x5Au);
  CHECK(s_reported_motor == &motor);
  s_protocol_report_result = false;
  CHECK(!s_fault_callback(0xA5u, &motor));
  CHECK(s_reported_fault_bits == 0xA5u);
  CHECK(s_reported_motor == &motor);

  ResetState();
  g_protocol_type = 99u;
  AppComm_Bootstrap();
  CHECK(CheckBootstrapOrder() == 0);
  CHECK(s_initialized_protocol == PROTOCOL_VECTOR);
  CHECK(g_protocol_type == PROTOCOL_VECTOR);

  puts("App communication bootstrap tests PASSED");
  return 0;
}
