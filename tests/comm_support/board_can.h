#ifndef COMM_TEST_BOARD_CAN_H
#define COMM_TEST_BOARD_CAN_H
#include "protocol_messages.h"
bool BSP_CAN_SendFrame(const CAN_Frame *frame);
#endif
