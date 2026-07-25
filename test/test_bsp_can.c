// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../Src/HAL/bsp/bsp_can.h"
#include "main.h"

extern bool BSP_CAN_IsTxReady(void);

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      return 1;                                                                \
    }                                                                          \
  } while (0)

typedef struct {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
} QueuedRxMessage;

FDCAN_HandleTypeDef hfdcan1;
static FDCAN_HandleTypeDef other_fdcan;
static unsigned deinit_count;
static unsigned init_count;
static unsigned filter_count;
static unsigned global_filter_count;
static unsigned start_count;
static unsigned stop_count;
static unsigned notification_count;
static unsigned tx_count;
static unsigned abort_count;
static unsigned tx_free_level_count;
static HAL_StatusTypeDef deinit_result;
static HAL_StatusTypeDef init_result;
static HAL_StatusTypeDef filter_result;
static HAL_StatusTypeDef global_filter_result;
static HAL_StatusTypeDef start_result;
static HAL_StatusTypeDef notification_result;
static HAL_StatusTypeDef tx_result;
static HAL_StatusTypeDef rx_message_result;
static uint32_t latest_tx_buffer;
static uint32_t tx_free_level;
static uint32_t last_notifications;
static uint32_t last_notification_buffers;
static uint32_t last_abort_mask;

static FDCAN_FilterTypeDef configured_filters[2];
static FDCAN_TxHeaderTypeDef last_tx_header;
static uint8_t last_tx_data[8];
static QueuedRxMessage rx_messages[8];
static unsigned rx_count;
static unsigned rx_index;
static FDCAN_TxEventFifoTypeDef tx_events[8];
static unsigned tx_event_count;
static unsigned tx_event_index;
static unsigned rx_callback_count;
static BSP_CAN_Frame last_rx_frame;

HAL_StatusTypeDef HAL_FDCAN_DeInit(FDCAN_HandleTypeDef *hfdcan) {
  CHECK(hfdcan == &hfdcan1);
  deinit_count++;
  return deinit_result;
}

HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan) {
  CHECK(hfdcan == &hfdcan1);
  init_count++;
  return init_result;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan,
                                         FDCAN_FilterTypeDef *filter) {
  CHECK(hfdcan == &hfdcan1);
  if (filter_count <
      sizeof(configured_filters) / sizeof(configured_filters[0])) {
    configured_filters[filter_count] = *filter;
  }
  filter_count++;

  return filter_result;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan,
                                               uint32_t non_matching_std,
                                               uint32_t non_matching_ext,
                                               uint32_t reject_remote_std,
                                               uint32_t reject_remote_ext) {
  CHECK(hfdcan == &hfdcan1);
  CHECK(non_matching_std == FDCAN_REJECT);
  CHECK(non_matching_ext == FDCAN_REJECT);
  CHECK(reject_remote_std == FDCAN_REJECT_REMOTE);
  CHECK(reject_remote_ext == FDCAN_REJECT_REMOTE);
  global_filter_count++;
  return global_filter_result;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan) {
  CHECK(hfdcan == &hfdcan1);
  start_count++;
  return start_result;
}

HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan) {
  CHECK(hfdcan == &hfdcan1);
  stop_count++;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan,
                                                 uint32_t notifications,
                                                 uint32_t buffers) {
  CHECK(hfdcan == &hfdcan1);
  notification_count++;
  last_notifications = notifications;
  last_notification_buffers = buffers;
  return notification_result;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan,
                                                FDCAN_TxHeaderTypeDef *header,
                                                uint8_t *data) {
  CHECK(hfdcan == &hfdcan1);
  tx_count++;
  last_tx_header = *header;
  memcpy(last_tx_data, data, sizeof(last_tx_data));
  return tx_result;
}

uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *hfdcan) {
  CHECK(hfdcan == &hfdcan1);
  return latest_tx_buffer;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *hfdcan) {
  CHECK(hfdcan == &hfdcan1);
  tx_free_level_count++;
  return tx_free_level;
}

HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *hfdcan,
                                           uint32_t buffer_mask) {
  CHECK(hfdcan == &hfdcan1);
  abort_count++;
  last_abort_mask = buffer_mask;
  return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *hfdcan,
                                      uint32_t fifo) {
  CHECK(hfdcan == &hfdcan1);
  CHECK(fifo == FDCAN_RX_FIFO0 || fifo == FDCAN_RX_FIFO1);
  return (rx_index < rx_count) ? (rx_count - rx_index) : 0U;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *header,
                                         uint8_t *data) {
  CHECK(hfdcan == &hfdcan1);
  CHECK(fifo == FDCAN_RX_FIFO0 || fifo == FDCAN_RX_FIFO1);
  if (rx_message_result != HAL_OK) {
    return rx_message_result;
  }
  *header = rx_messages[rx_index].header;
  memcpy(data, rx_messages[rx_index].data, sizeof(rx_messages[rx_index].data));
  rx_index++;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *hfdcan,
                                       FDCAN_TxEventFifoTypeDef *event) {
  CHECK(hfdcan == &hfdcan1);
  if (tx_event_index >= tx_event_count) {
    return HAL_ERROR;
  }
  *event = tx_events[tx_event_index++];
  return HAL_OK;
}

#define BSP_LOG_H
#define LOGERROR(...) ((void)0)
#define LOGINFO(...) ((void)0)
#include "../Src/HAL/bsp/bsp_can.c"

static void capture_rx_frame(const BSP_CAN_Frame *frame) {
  rx_callback_count++;
  if (frame != NULL) {
    last_rx_frame = *frame;
  }
}

static void reset_fixture(void) {
  memset(&hfdcan1, 0, sizeof(hfdcan1));
  memset(&other_fdcan, 0, sizeof(other_fdcan));
  deinit_count = 0U;
  init_count = 0U;
  filter_count = 0U;
  global_filter_count = 0U;
  start_count = 0U;
  stop_count = 0U;
  notification_count = 0U;
  tx_count = 0U;
  abort_count = 0U;
  tx_free_level_count = 0U;
  deinit_result = HAL_OK;
  init_result = HAL_OK;
  filter_result = HAL_OK;
  global_filter_result = HAL_OK;
  start_result = HAL_OK;
  notification_result = HAL_OK;
  tx_result = HAL_OK;
  rx_message_result = HAL_OK;
  latest_tx_buffer = 0x04U;
  tx_free_level = 0U;
  last_notifications = 0U;
  last_notification_buffers = 0U;
  last_abort_mask = 0U;

  memset(configured_filters, 0, sizeof(configured_filters));
  memset(&last_tx_header, 0, sizeof(last_tx_header));
  memset(last_tx_data, 0, sizeof(last_tx_data));
  memset(rx_messages, 0, sizeof(rx_messages));
  rx_count = 0U;
  rx_index = 0U;
  memset(tx_events, 0, sizeof(tx_events));
  tx_event_count = 0U;
  tx_event_index = 0U;
  rx_callback_count = 0U;
  memset(&last_rx_frame, 0, sizeof(last_rx_frame));
  s_tracked_tx_next_marker = 1U;
  s_tracked_tx_pending_marker = 0U;
  s_tracked_tx_pending_buffer_mask = 0U;
  s_tracked_tx_completed_marker = 0U;
  (void)BSP_CAN_SetRxCallback(NULL);
}

static BSP_CAN_Frame make_data_frame(uint32_t id, bool extended, uint8_t dlc) {
  BSP_CAN_Frame frame = {
      .id = id,
      .dlc = dlc,
      .is_extended = extended,
      .is_rtr = false,
  };
  for (uint8_t i = 0U; i < sizeof(frame.data); ++i) {
    frame.data[i] = (uint8_t)(0xA0U + i);
  }
  return frame;
}

static void queue_rx_message(uint32_t id, bool extended, bool rtr,
                             uint8_t dlc) {
  QueuedRxMessage *message = &rx_messages[rx_count++];
  message->header.Identifier = id;
  message->header.IdType = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  message->header.RxFrameType = rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  message->header.DataLength = ((uint32_t)dlc << 16);
  for (uint8_t i = 0U; i < sizeof(message->data); ++i) {
    message->data[i] = (uint8_t)(0x10U + i);
  }
}

static int test_init_requires_registered_rx_callback(void) {
  reset_fixture();
  CHECK(!BSP_CAN_Init(BSP_CAN_BAUD_1M));
  CHECK(deinit_count == 0U);
  CHECK(start_count == 0U);
  return 0;
}

static int test_init_configures_500k_baud_filters_and_notifications(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  CHECK(BSP_CAN_Init(BSP_CAN_BAUD_500K));
  CHECK(deinit_count == 1U);
  CHECK(init_count == 1U);
  CHECK(hfdcan1.Init.NominalPrescaler == 12U);
  CHECK(hfdcan1.Init.NominalSyncJumpWidth == 7U);
  CHECK(hfdcan1.Init.NominalTimeSeg1 == 20U);
  CHECK(hfdcan1.Init.NominalTimeSeg2 == 7U);
  CHECK(hfdcan1.Init.DataPrescaler == 12U);
  CHECK(hfdcan1.Init.StdFiltersNbr == 1U);
  CHECK(hfdcan1.Init.ExtFiltersNbr == 1U);
  CHECK(filter_count == 2U);
  CHECK(configured_filters[0].IdType == FDCAN_EXTENDED_ID);
  CHECK(configured_filters[0].FilterType == FDCAN_FILTER_RANGE);
  CHECK(configured_filters[0].FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
  CHECK(configured_filters[0].FilterID1 == 0U);
  CHECK(configured_filters[0].FilterID2 == 0x1FFFFFFFU);
  CHECK(configured_filters[1].IdType == FDCAN_STANDARD_ID);
  CHECK(configured_filters[1].FilterType == FDCAN_FILTER_RANGE);
  CHECK(configured_filters[1].FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
  CHECK(configured_filters[1].FilterID1 == 0U);
  CHECK(configured_filters[1].FilterID2 == 0x7FFU);
  CHECK(global_filter_count == 1U);
  CHECK(start_count == 1U);
  CHECK(notification_count == 1U);
  CHECK(last_notifications ==
        (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
         FDCAN_IT_TX_EVT_FIFO_NEW_DATA));
  CHECK(last_notification_buffers == 0U);
  return 0;
}

static int test_init_rejects_unknown_baudrate_before_hal_reinit(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  CHECK(!BSP_CAN_Init((BSP_CAN_BaudrateId)99U));
  CHECK(deinit_count == 0U);
  CHECK(filter_count == 0U);
  return 0;
}

static int test_init_stops_can_when_notification_activation_fails(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  notification_result = HAL_ERROR;
  CHECK(!BSP_CAN_Init(BSP_CAN_BAUD_1M));
  CHECK(start_count == 1U);
  CHECK(notification_count == 1U);
  CHECK(stop_count == 1U);
  return 0;
}

static int test_init_propagates_filter_failures_before_start(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  global_filter_result = HAL_ERROR;
  CHECK(!BSP_CAN_Init(BSP_CAN_BAUD_1M));
  CHECK(filter_count == 2U);
  CHECK(global_filter_count == 1U);
  CHECK(start_count == 0U);
  return 0;
}

static int test_send_frame_maps_standard_identifier_dlc_and_payload(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x321U, false, 6U);
  CHECK(BSP_CAN_SendFrame(&frame));
  CHECK(tx_count == 1U);
  CHECK(last_tx_header.Identifier == 0x321U);
  CHECK(last_tx_header.IdType == FDCAN_STANDARD_ID);
  CHECK(last_tx_header.TxFrameType == FDCAN_DATA_FRAME);
  CHECK(last_tx_header.DataLength == FDCAN_DLC_BYTES_6);
  CHECK(last_tx_header.TxEventFifoControl == FDCAN_NO_TX_EVENTS);
  CHECK(last_tx_header.MessageMarker == 0U);
  CHECK(memcmp(last_tx_data, frame.data, sizeof(frame.data)) == 0);
  return 0;
}

static int test_send_frame_maps_extended_identifier_and_empty_dlc(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x1ABCDE3U, true, 0U);
  CHECK(BSP_CAN_SendFrame(&frame));
  CHECK(last_tx_header.Identifier == 0x1ABCDE3U);
  CHECK(last_tx_header.IdType == FDCAN_EXTENDED_ID);
  CHECK(last_tx_header.DataLength == FDCAN_DLC_BYTES_0);
  return 0;
}

static int test_send_frame_rejects_invalid_dlc_rtr_and_identifiers(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x123U, false, 9U);
  CHECK(!BSP_CAN_SendFrame(&frame));
  frame = make_data_frame(0x123U, false, 1U);
  frame.is_rtr = true;
  CHECK(!BSP_CAN_SendFrame(&frame));
  frame = make_data_frame(0x800U, false, 1U);
  CHECK(!BSP_CAN_SendFrame(&frame));
  frame = make_data_frame(0x20000000U, true, 1U);
  CHECK(!BSP_CAN_SendFrame(&frame));
  CHECK(!BSP_CAN_SendFrame(NULL));
  CHECK(tx_count == 0U);
  return 0;
}

static int
test_rx_fifo_callback_delivers_extended_and_standard_data_frames(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  queue_rx_message(0x1ABCDE3U, true, false, 8U);
  HAL_FDCAN_RxFifo0Callback(&hfdcan1, 0U);
  CHECK(rx_callback_count == 0U);
  HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  CHECK(rx_callback_count == 1U);
  CHECK(last_rx_frame.id == 0x1ABCDE3U);
  CHECK(last_rx_frame.dlc == 8U);
  CHECK(last_rx_frame.is_extended);
  CHECK(!last_rx_frame.is_rtr);
  CHECK(memcmp(last_rx_frame.data, rx_messages[0].data, 8U) == 0);

  queue_rx_message(0x321U, false, false, 4U);
  HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  CHECK(rx_callback_count == 2U);
  CHECK(last_rx_frame.id == 0x321U);
  CHECK(last_rx_frame.dlc == 4U);
  CHECK(!last_rx_frame.is_extended);
  CHECK(!last_rx_frame.is_rtr);
  CHECK(memcmp(last_rx_frame.data, rx_messages[1].data, 4U) == 0);
  return 0;
}
static int test_rx_fifo_callback_ignores_wrong_handle_and_invalid_frames(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  queue_rx_message(0x123U, false, true, 1U);
  queue_rx_message(0x123U, false, false, 9U);
  queue_rx_message(0x800U, false, false, 1U);
  queue_rx_message(0x20000000U, true, false, 1U);
  HAL_FDCAN_RxFifo1Callback(&other_fdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  CHECK(rx_index == 0U);
  HAL_FDCAN_RxFifo1Callback(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
  CHECK(rx_callback_count == 0U);
  CHECK(rx_index == rx_count);
  return 0;
}

static int test_rx_fifo_callback_stops_when_get_message_fails(void) {
  reset_fixture();
  CHECK(BSP_CAN_SetRxCallback(capture_rx_frame));
  queue_rx_message(0x123U, false, false, 1U);
  rx_message_result = HAL_ERROR;
  HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  CHECK(rx_callback_count == 0U);
  CHECK(rx_index == 0U);
  return 0;
}

static int test_is_tx_ready_returns_false_when_tx_fifo_has_no_free_slots(void) {
  reset_fixture();
  tx_free_level = 0U;

  CHECK(!BSP_CAN_IsTxReady());

  CHECK(tx_free_level_count == 1U);
  return 0;
}

static int test_is_tx_ready_returns_true_when_tx_fifo_has_free_slots(void) {
  reset_fixture();
  tx_free_level = 2U;

  CHECK(BSP_CAN_IsTxReady());

  CHECK(tx_free_level_count == 1U);
  return 0;
}
static int
test_tracked_send_returns_ticket_and_completes_from_matching_event(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x123U, false, 2U);
  BSP_CAN_TxTicket ticket = {0};
  CHECK(BSP_CAN_SendTrackedFrame(&frame, &ticket));
  CHECK(tx_count == 1U);
  CHECK(ticket.marker == 1U);
  CHECK(ticket.tx_buffer_mask == latest_tx_buffer);
  CHECK(last_tx_header.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
  CHECK(last_tx_header.MessageMarker == 1U);
  CHECK(!BSP_CAN_TxTicketIsComplete(&ticket));
  tx_events[tx_event_count++].MessageMarker = 0xAAU;
  tx_events[tx_event_count++].MessageMarker = ticket.marker;
  HAL_FDCAN_TxEventFifoCallback(&hfdcan1, FDCAN_IT_TX_EVT_FIFO_NEW_DATA);
  CHECK(BSP_CAN_TxTicketIsComplete(&ticket));
  CHECK(!BSP_CAN_TxTicketIsComplete(&ticket));
  return 0;
}

static int test_tracked_send_rejects_second_pending_request(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x123U, false, 2U);
  BSP_CAN_TxTicket first = {0};
  BSP_CAN_TxTicket second = {0};
  CHECK(BSP_CAN_SendTrackedFrame(&frame, &first));
  CHECK(!BSP_CAN_SendTrackedFrame(&frame, &second));
  CHECK(tx_count == 1U);
  CHECK(second.marker == 0U);
  return 0;
}

static int test_cancel_tracked_send_aborts_matching_pending_request(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x123U, false, 2U);
  BSP_CAN_TxTicket ticket = {0};
  CHECK(BSP_CAN_SendTrackedFrame(&frame, &ticket));
  BSP_CAN_CancelTrackedSend(&ticket);
  CHECK(abort_count == 1U);
  CHECK(last_abort_mask == ticket.tx_buffer_mask);
  CHECK(!BSP_CAN_TxTicketIsComplete(&ticket));
  return 0;
}

static int test_cancel_tracked_send_ignores_null_and_mismatched_tickets(void) {
  reset_fixture();
  BSP_CAN_Frame frame = make_data_frame(0x123U, false, 2U);
  BSP_CAN_TxTicket ticket = {0};
  BSP_CAN_TxTicket wrong = {.marker = 0x55U, .tx_buffer_mask = 0x40U};
  BSP_CAN_CancelTrackedSend(NULL);
  CHECK(BSP_CAN_SendTrackedFrame(&frame, &ticket));
  BSP_CAN_CancelTrackedSend(&wrong);
  CHECK(abort_count == 0U);
  return 0;
}

int main(void) {
  int failures = test_init_requires_registered_rx_callback();
  failures += test_init_configures_500k_baud_filters_and_notifications();
  failures += test_init_rejects_unknown_baudrate_before_hal_reinit();
  failures += test_init_stops_can_when_notification_activation_fails();
  failures += test_init_propagates_filter_failures_before_start();
  failures += test_send_frame_maps_standard_identifier_dlc_and_payload();
  failures += test_send_frame_maps_extended_identifier_and_empty_dlc();
  failures += test_send_frame_rejects_invalid_dlc_rtr_and_identifiers();
  failures +=
      test_rx_fifo_callback_delivers_extended_and_standard_data_frames();
  failures += test_rx_fifo_callback_ignores_wrong_handle_and_invalid_frames();
  failures += test_rx_fifo_callback_stops_when_get_message_fails();
  failures += test_is_tx_ready_returns_false_when_tx_fifo_has_no_free_slots();
  failures += test_is_tx_ready_returns_true_when_tx_fifo_has_free_slots();
  failures +=
      test_tracked_send_returns_ticket_and_completes_from_matching_event();
  failures += test_tracked_send_rejects_second_pending_request();
  failures += test_cancel_tracked_send_aborts_matching_pending_request();
  failures += test_cancel_tracked_send_ignores_null_and_mismatched_tickets();
  if (failures == 0) {
    printf("All BSP CAN direct tests PASSED\n");
  }
  return failures;
}
