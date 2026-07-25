#ifndef TEST_APP_COMM_BOOTSTRAP_CAN_TRANSPORT_H
#define TEST_APP_COMM_BOOTSTRAP_CAN_TRANSPORT_H

#include <stdbool.h>

#include "transport.h"

bool CAN_Transport_Init(void);
const TransportInterface *CAN_Transport_GetInterface(void);

#endif /* TEST_APP_COMM_BOOTSTRAP_CAN_TRANSPORT_H */
