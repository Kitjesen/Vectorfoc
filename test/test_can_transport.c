#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_can.h"
#include "can_transport.h"

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static BSP_CAN_RxCallback_t s_bsp_rx_callback;
static unsigned s_set_rx_callback_count;
static bool s_set_rx_callback_result = true;
static unsigned s_rx_callback_count;
static TransportFrame s_last_rx_frame;
static unsigned s_send_count;
static BSP_CAN_Frame s_last_sent_frame;
static bool s_send_result = true;
static unsigned s_tracked_send_count;
static BSP_CAN_Frame s_last_tracked_sent_frame;
static bool s_tracked_send_result = true;
static BSP_CAN_TxTicket s_tracked_ticket_to_return = {0x1234U, 0x04U};
static unsigned s_complete_check_count;
static BSP_CAN_TxTicket s_last_complete_ticket;
static bool s_complete_result;
static unsigned s_cancel_count;
static BSP_CAN_TxTicket s_last_cancelled_ticket;
static bool s_tx_ready;

static void reset_fixture(void) {
  s_bsp_rx_callback = NULL;
  s_set_rx_callback_count = 0U;
  s_set_rx_callback_result = true;
  s_rx_callback_count = 0U;
  memset(&s_last_rx_frame, 0, sizeof(s_last_rx_frame));
  s_send_count = 0U;
  memset(&s_last_sent_frame, 0, sizeof(s_last_sent_frame));
  s_send_result = true;
  s_tracked_send_count = 0U;
  memset(&s_last_tracked_sent_frame, 0, sizeof(s_last_tracked_sent_frame));
  s_tracked_send_result = true;
  s_tracked_ticket_to_return.marker = 0x1234U;
  s_tracked_ticket_to_return.tx_buffer_mask = 0x04U;
  s_complete_check_count = 0U;
  memset(&s_last_complete_ticket, 0, sizeof(s_last_complete_ticket));
  s_complete_result = false;
  s_cancel_count = 0U;
  memset(&s_last_cancelled_ticket, 0, sizeof(s_last_cancelled_ticket));
  s_tx_ready = false;
}

static TransportFrame make_transport_frame(void) {
  TransportFrame frame = {0};
  frame.type = TRANSPORT_CAN;
  frame.id = 0x1ABCDE3U;
  frame.len = 8U;
  frame.is_extended = true;
  for (uint8_t i = 0U; i < 8U; ++i) {
    frame.data[i] = (uint8_t)(0xA0U + i);
  }
  return frame;
}

static void capture_rx_frame(const TransportFrame *frame) {
  s_rx_callback_count++;
  if (frame != NULL) {
    s_last_rx_frame = *frame;
  }
}

static int test_init_registers_bsp_rx_callback(void) {
  reset_fixture();

  CHECK(CAN_Transport_Init());
  CHECK(s_set_rx_callback_count == 1U);
  CHECK(s_bsp_rx_callback != NULL);
  return 0;
}

static int test_register_rx_callback_accepts_non_null_callback(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();

  CHECK(transport->register_rx_callback(capture_rx_frame));
  return 0;
}

static int test_register_rx_callback_rejects_null_callback(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();

  CHECK(!transport->register_rx_callback(NULL));
  return 0;
}

static int test_bsp_rx_callback_maps_extended_frame_to_transport_frame(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  BSP_CAN_Frame frame = {0};
  frame.id = 0x1ABCDE3U;
  frame.dlc = 8U;
  frame.is_extended = true;
  frame.is_rtr = false;
  for (uint8_t i = 0U; i < frame.dlc; ++i) {
    frame.data[i] = (uint8_t)(0x10U + i);
  }

  CHECK(CAN_Transport_Init());
  CHECK(transport->register_rx_callback(capture_rx_frame));
  s_bsp_rx_callback(&frame);

  CHECK(s_rx_callback_count == 1U);
  CHECK(s_last_rx_frame.type == TRANSPORT_CAN);
  CHECK(s_last_rx_frame.id == frame.id);
  CHECK(s_last_rx_frame.len == frame.dlc);
  CHECK(s_last_rx_frame.is_extended == frame.is_extended);
  CHECK(memcmp(s_last_rx_frame.data, frame.data, frame.dlc) == 0);
  return 0;
}

static int test_bsp_rx_callback_maps_standard_frame_to_transport_frame(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  BSP_CAN_Frame frame = {
      .id = 0x321U,
      .dlc = 4U,
      .is_extended = false,
      .is_rtr = false,
      .data = {0x11U, 0x22U, 0x33U, 0x44U},
  };

  CHECK(CAN_Transport_Init());
  CHECK(transport->register_rx_callback(capture_rx_frame));
  s_bsp_rx_callback(&frame);

  CHECK(s_rx_callback_count == 1U);
  CHECK(s_last_rx_frame.type == TRANSPORT_CAN);
  CHECK(s_last_rx_frame.id == frame.id);
  CHECK(s_last_rx_frame.len == frame.dlc);
  CHECK(!s_last_rx_frame.is_extended);
  CHECK(memcmp(s_last_rx_frame.data, frame.data, frame.dlc) == 0);
  return 0;
}

static int test_send_maps_transport_frame_to_bsp_frame(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  TransportFrame frame = make_transport_frame();

  CHECK(transport->send(&frame));

  CHECK(s_send_count == 1U);
  CHECK(s_last_sent_frame.id == frame.id);
  CHECK(s_last_sent_frame.dlc == 8U);
  CHECK(s_last_sent_frame.is_extended == frame.is_extended);
  CHECK(!s_last_sent_frame.is_rtr);
  CHECK(memcmp(s_last_sent_frame.data, frame.data, 8U) == 0);
  return 0;
}

static int test_send_tracked_maps_frame_and_returns_ticket(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  TransportFrame frame = make_transport_frame();
  TransportTxTicket ticket = {0};

  CHECK(transport->send_tracked(&frame, &ticket));

  CHECK(s_tracked_send_count == 1U);
  CHECK(s_last_tracked_sent_frame.id == frame.id);
  CHECK(s_last_tracked_sent_frame.dlc == 8U);
  CHECK(s_last_tracked_sent_frame.is_extended == frame.is_extended);
  CHECK(!s_last_tracked_sent_frame.is_rtr);
  CHECK(memcmp(s_last_tracked_sent_frame.data, frame.data, 8U) == 0);
  CHECK(ticket.marker == s_tracked_ticket_to_return.marker);
  CHECK(ticket.tx_buffer_mask == s_tracked_ticket_to_return.tx_buffer_mask);
  return 0;
}

static int test_tx_ticket_is_complete_adapts_ticket_to_bsp(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  TransportTxTicket ticket = {.marker = 0xCAFEU, .tx_buffer_mask = 0x02U};
  s_complete_result = true;

  CHECK(transport->tx_ticket_is_complete(&ticket));

  CHECK(s_complete_check_count == 1U);
  CHECK(s_last_complete_ticket.marker == ticket.marker);
  CHECK(s_last_complete_ticket.tx_buffer_mask == ticket.tx_buffer_mask);
  return 0;
}

static int test_cancel_tracked_tx_adapts_ticket_to_bsp(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  TransportTxTicket ticket = {.marker = 0xBEEFU, .tx_buffer_mask = 0x08U};

  transport->cancel_tracked_tx(&ticket);

  CHECK(s_cancel_count == 1U);
  CHECK(s_last_cancelled_ticket.marker == ticket.marker);
  CHECK(s_last_cancelled_ticket.tx_buffer_mask == ticket.tx_buffer_mask);
  return 0;
}

static int test_is_tx_ready_reports_hal_mailbox_availability(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();

  s_tx_ready = true;
  CHECK(transport->is_tx_ready());
  s_tx_ready = false;
  CHECK(!transport->is_tx_ready());
  return 0;
}

static int test_init_failure_is_propagated(void) {
  reset_fixture();
  s_set_rx_callback_result = false;

  CHECK(!CAN_Transport_Init());
  CHECK(s_set_rx_callback_count == 1U);
  return 0;
}

static int test_reinitialization_preserves_registered_upper_callback(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  BSP_CAN_Frame frame = {
      .id = 0x123U,
      .dlc = 1U,
      .is_extended = false,
  };

  CHECK(transport->register_rx_callback(capture_rx_frame));
  CHECK(CAN_Transport_Init());
  s_bsp_rx_callback(&frame);

  CHECK(s_rx_callback_count == 1U);
  return 0;
}

static int test_rx_rejects_rtr_oversized_and_invalid_identifiers(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  BSP_CAN_Frame frame = {
      .id = 0x123U,
      .dlc = 1U,
      .is_extended = false,
      .is_rtr = true,
  };

  CHECK(CAN_Transport_Init());
  CHECK(transport->register_rx_callback(capture_rx_frame));
  s_bsp_rx_callback(&frame);

  frame.is_rtr = false;
  frame.dlc = 9U;
  s_bsp_rx_callback(&frame);

  frame.dlc = 1U;
  frame.id = 0x800U;
  s_bsp_rx_callback(&frame);

  frame.is_extended = true;
  frame.id = 0x20000000U;
  s_bsp_rx_callback(&frame);

  CHECK(s_rx_callback_count == 0U);
  return 0;
}

static int test_send_rejects_wrong_type_oversize_and_invalid_identifier(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  TransportFrame frame = make_transport_frame();

  frame.type = TRANSPORT_USB;
  CHECK(!transport->send(&frame));

  frame.type = TRANSPORT_CAN;
  frame.len = 9U;
  CHECK(!transport->send(&frame));

  frame.len = 1U;
  frame.is_extended = false;
  frame.id = 0x800U;
  CHECK(!transport->send(&frame));

  frame.is_extended = true;
  frame.id = 0x20000000U;
  CHECK(!transport->send(&frame));

  CHECK(s_send_count == 0U);
  return 0;
}

static int test_failed_tracked_send_clears_ticket(void) {
  reset_fixture();
  const TransportInterface *transport = CAN_Transport_GetInterface();
  TransportFrame frame = make_transport_frame();
  TransportTxTicket ticket = {
      .marker = 0xDEADU,
      .tx_buffer_mask = 0x55U,
  };
  s_tracked_send_result = false;

  CHECK(!transport->send_tracked(&frame, &ticket));

  CHECK(ticket.marker == 0U);
  CHECK(ticket.tx_buffer_mask == 0U);
  return 0;
}
int main(void) {
  int failures = test_init_registers_bsp_rx_callback();
  failures += test_init_failure_is_propagated();
  failures += test_reinitialization_preserves_registered_upper_callback();
  failures += test_register_rx_callback_accepts_non_null_callback();
  failures += test_register_rx_callback_rejects_null_callback();
  failures += test_bsp_rx_callback_maps_extended_frame_to_transport_frame();
  failures += test_bsp_rx_callback_maps_standard_frame_to_transport_frame();
  failures += test_send_maps_transport_frame_to_bsp_frame();
  failures += test_send_tracked_maps_frame_and_returns_ticket();
  failures += test_tx_ticket_is_complete_adapts_ticket_to_bsp();
  failures += test_cancel_tracked_tx_adapts_ticket_to_bsp();
  failures += test_is_tx_ready_reports_hal_mailbox_availability();
  failures += test_rx_rejects_rtr_oversized_and_invalid_identifiers();
  failures += test_send_rejects_wrong_type_oversize_and_invalid_identifier();
  failures += test_failed_tracked_send_clears_ticket();
  if (failures == 0) {
    printf("All CAN transport tests PASSED\n");
  }
  return failures;
}

bool BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t cb) {
  s_set_rx_callback_count++;
  s_bsp_rx_callback = cb;
  return s_set_rx_callback_result;
}

bool BSP_CAN_SendFrame(const BSP_CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  s_send_count++;
  s_last_sent_frame = *frame;
  return s_send_result;
}

bool BSP_CAN_SendTrackedFrame(const BSP_CAN_Frame *frame,
                              BSP_CAN_TxTicket *ticket) {
  if (frame == NULL || ticket == NULL) {
    return false;
  }
  s_tracked_send_count++;
  s_last_tracked_sent_frame = *frame;
  if (!s_tracked_send_result) {
    return false;
  }
  *ticket = s_tracked_ticket_to_return;
  return true;
}

bool BSP_CAN_TxTicketIsComplete(const BSP_CAN_TxTicket *ticket) {
  if (ticket == NULL) {
    return false;
  }
  s_complete_check_count++;
  s_last_complete_ticket = *ticket;
  return s_complete_result;
}

void BSP_CAN_CancelTrackedSend(const BSP_CAN_TxTicket *ticket) {
  if (ticket == NULL) {
    return;
  }
  s_cancel_count++;
  s_last_cancelled_ticket = *ticket;
}

bool BSP_CAN_IsTxReady(void) { return s_tx_ready; }
