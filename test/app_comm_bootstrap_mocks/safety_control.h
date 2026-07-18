#ifndef TEST_APP_COMM_BOOTSTRAP_SAFETY_CONTROL_H
#define TEST_APP_COMM_BOOTSTRAP_SAFETY_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef bool (*SafetyFaultCallback)(uint32_t fault_bits, void *motor);

void Safety_RegisterFaultCallback(SafetyFaultCallback callback);

#endif /* TEST_APP_COMM_BOOTSTRAP_SAFETY_CONTROL_H */
