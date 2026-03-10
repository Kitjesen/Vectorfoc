#ifndef TEST_MOCK_BSP_CAN_H
#define TEST_MOCK_BSP_CAN_H

#include "protocol_types.h"
#include <stdbool.h>

bool BSP_CAN_SendFrame(const CAN_Frame *frame);

#endif
