#ifndef TEST_APP_COMM_BOOTSTRAP_MANAGER_H
#define TEST_APP_COMM_BOOTSTRAP_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "motor.h"
#include "protocol_types.h"
#include "transport.h"

bool Protocol_RegisterTransport(const TransportInterface *transport);
void Protocol_Init(ProtocolType default_protocol);
bool Protocol_ReportFaultCallback(uint32_t fault_bits, MOTOR_DATA *motor);

#endif /* TEST_APP_COMM_BOOTSTRAP_MANAGER_H */
