#ifndef TEST_MOCK_MOTOR_H
#define TEST_MOCK_MOTOR_H

#include <stdint.h>

extern uint8_t g_can_id;
extern uint8_t g_can_baudrate;
extern uint8_t g_protocol_type;
extern uint32_t g_can_timeout_ms;

typedef enum {
  CONTROL_MODE_OPEN = 0,
  CONTROL_MODE_TORQUE = 1,
  CONTROL_MODE_VELOCITY = 2,
  CONTROL_MODE_POSITION = 3,
  CONTROL_MODE_VELOCITY_RAMP = 4,
  CONTROL_MODE_POSITION_RAMP = 5,
  CONTROL_MODE_MIT = 6,
} CONTROL_MODE;

#endif
