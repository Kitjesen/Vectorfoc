// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file can_transport.c
 * @brief Adapter between the generic transport seam and the FDCAN BSP.
 */
#include "can_transport.h"

#include "bsp_can.h"
#include <stddef.h>
#include <string.h>

static TransportRxCallback s_rx_callback;

static bool CAN_Transport_ToBspFrame(const TransportFrame *source,
                                     BSP_CAN_Frame *destination) {
  if (source == NULL || destination == NULL || source->type != TRANSPORT_CAN ||
      source->len > sizeof(destination->data)) {
    return false;
  }
  const uint32_t max_id = source->is_extended ? 0x1FFFFFFFU : 0x7FFU;
  if (source->id > max_id) {
    return false;
  }

  memset(destination, 0, sizeof(*destination));
  destination->id = source->id;
  destination->dlc = source->len;
  destination->is_extended = source->is_extended;
  memcpy(destination->data, source->data, source->len);
  return true;
}

static void CAN_Transport_DispatchRx(const BSP_CAN_Frame *frame) {
  if (frame == NULL || s_rx_callback == NULL || frame->is_rtr ||
      frame->dlc > sizeof(frame->data)) {
    return;
  }
  const uint32_t max_id = frame->is_extended ? 0x1FFFFFFFU : 0x7FFU;
  if (frame->id > max_id) {
    return;
  }

  TransportFrame transport_frame = {
      .type = TRANSPORT_CAN,
      .id = frame->id,
      .len = frame->dlc,
      .is_extended = frame->is_extended,
  };
  memcpy(transport_frame.data, frame->data, frame->dlc);
  s_rx_callback(&transport_frame);
}

static bool CAN_Transport_Send(const TransportFrame *frame) {
  BSP_CAN_Frame bsp_frame;
  if (!CAN_Transport_ToBspFrame(frame, &bsp_frame)) {
    return false;
  }
  return BSP_CAN_SendFrame(&bsp_frame);
}

static bool CAN_Transport_SendTracked(const TransportFrame *frame,
                                      TransportTxTicket *ticket) {
  if (ticket == NULL) {
    return false;
  }
  memset(ticket, 0, sizeof(*ticket));
  BSP_CAN_Frame bsp_frame;
  BSP_CAN_TxTicket bsp_ticket = {0};
  if (!CAN_Transport_ToBspFrame(frame, &bsp_frame) ||
      !BSP_CAN_SendTrackedFrame(&bsp_frame, &bsp_ticket)) {
    return false;
  }
  ticket->marker = bsp_ticket.marker;
  ticket->tx_buffer_mask = bsp_ticket.tx_buffer_mask;
  return true;
}

static bool CAN_Transport_TxTicketIsComplete(const TransportTxTicket *ticket) {
  if (ticket == NULL) {
    return false;
  }
  BSP_CAN_TxTicket bsp_ticket = {
      .marker = ticket->marker,
      .tx_buffer_mask = ticket->tx_buffer_mask,
  };
  return BSP_CAN_TxTicketIsComplete(&bsp_ticket);
}

static void CAN_Transport_CancelTrackedTx(const TransportTxTicket *ticket) {
  if (ticket == NULL) {
    return;
  }
  BSP_CAN_TxTicket bsp_ticket = {
      .marker = ticket->marker,
      .tx_buffer_mask = ticket->tx_buffer_mask,
  };
  BSP_CAN_CancelTrackedSend(&bsp_ticket);
}

static bool CAN_Transport_RegisterRxCallback(TransportRxCallback callback) {
  s_rx_callback = callback;
  return callback != NULL;
}

static bool CAN_Transport_IsTxReady(void) { return BSP_CAN_IsTxReady(); }

static const TransportInterface s_can_transport = {
    .send = CAN_Transport_Send,
    .send_tracked = CAN_Transport_SendTracked,
    .tx_ticket_is_complete = CAN_Transport_TxTicketIsComplete,
    .cancel_tracked_tx = CAN_Transport_CancelTrackedTx,
    .register_rx_callback = CAN_Transport_RegisterRxCallback,
    .is_tx_ready = CAN_Transport_IsTxReady,
    .type = TRANSPORT_CAN,
};

const TransportInterface *CAN_Transport_GetInterface(void) {
  return &s_can_transport;
}

bool CAN_Transport_Init(void) {
  return BSP_CAN_SetRxCallback(CAN_Transport_DispatchRx);
}