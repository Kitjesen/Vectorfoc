#ifndef TEST_APP_COMM_BOOTSTRAP_MOTOR_H
#define TEST_APP_COMM_BOOTSTRAP_MOTOR_H

#include <stdint.h>

typedef struct MOTOR_DATA_s {
  uint32_t test_tag;
} MOTOR_DATA;

extern uint8_t g_protocol_type;
extern uint8_t g_can_baudrate;

#endif /* TEST_APP_COMM_BOOTSTRAP_MOTOR_H */
