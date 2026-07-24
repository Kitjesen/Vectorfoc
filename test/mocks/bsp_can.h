#ifndef TEST_MOCK_BSP_CAN_H
#define TEST_MOCK_BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t id;
  uint8_t data[8];
  uint8_t dlc;
  bool is_extended;
  bool is_rtr;
} BSP_CAN_Frame;

typedef struct {
  uint32_t marker;
  uint32_t tx_buffer_mask;
} BSP_CAN_TxTicket;

typedef void (*BSP_CAN_RxCallback_t)(const BSP_CAN_Frame *frame);

bool BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t cb);
bool BSP_CAN_IsTxReady(void);
bool BSP_CAN_SendFrame(const BSP_CAN_Frame *frame);
bool BSP_CAN_SendTrackedFrame(const BSP_CAN_Frame *frame,
                              BSP_CAN_TxTicket *ticket);
bool BSP_CAN_TxTicketIsComplete(const BSP_CAN_TxTicket *ticket);
void BSP_CAN_CancelTrackedSend(const BSP_CAN_TxTicket *ticket);

#endif
