#ifndef TEST_APP_COMM_BOOTSTRAP_BSP_CAN_H
#define TEST_APP_COMM_BOOTSTRAP_BSP_CAN_H

#include <stdbool.h>

typedef enum {
  BSP_CAN_BAUD_1M = 0U,
  BSP_CAN_BAUD_500K = 1U,
  BSP_CAN_BAUD_250K = 2U,
} BSP_CAN_BaudrateId;

bool BSP_CAN_Init(BSP_CAN_BaudrateId baudrate_id);

#endif /* TEST_APP_COMM_BOOTSTRAP_BSP_CAN_H */
