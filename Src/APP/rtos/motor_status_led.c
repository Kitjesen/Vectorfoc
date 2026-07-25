// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "motor_status_led.h"

#define MOTOR_STATUS_LED_IDLE_PERIOD_TICKS 200u
#define MOTOR_STATUS_LED_IDLE_HALF_PERIOD_TICKS 100u
#define MOTOR_STATUS_LED_GUARD_PERIOD_TICKS 20u
#define MOTOR_STATUS_LED_GUARD_HALF_PERIOD_TICKS 10u

void MotorStatusLed_Init(MotorStatusLedState *led) {
  if (led == NULL) {
    return;
  }

  led->idle_tick = 0u;
  led->guard_tick = 0u;
}

uint8_t MotorStatusLed_Update(MotorStatusLedState *led, STATE_MODE mode,
                              bool has_active_fault) {
  if (mode == STATE_MODE_IDLE || mode == STATE_MODE_DETECTING) {
    uint32_t tick = led != NULL ? led->idle_tick : 0u;
    uint8_t color = tick < MOTOR_STATUS_LED_IDLE_HALF_PERIOD_TICKS
                        ? MOTOR_STATUS_LED_COLOR_PURPLE
                        : MOTOR_STATUS_LED_COLOR_BLACK;
    if (led != NULL) {
      led->idle_tick = (tick + 1u) % MOTOR_STATUS_LED_IDLE_PERIOD_TICKS;
    }
    return color;
  }

  if (has_active_fault) {
    return MOTOR_STATUS_LED_COLOR_RED;
  }

  if (mode == STATE_MODE_RUNNING) {
    return MOTOR_STATUS_LED_COLOR_CYAN;
  }

  uint32_t tick = led != NULL ? led->guard_tick : 0u;
  uint8_t color = tick < MOTOR_STATUS_LED_GUARD_HALF_PERIOD_TICKS
                      ? MOTOR_STATUS_LED_COLOR_RED
                      : MOTOR_STATUS_LED_COLOR_BLACK;
  if (led != NULL) {
    led->guard_tick = (tick + 1u) % MOTOR_STATUS_LED_GUARD_PERIOD_TICKS;
  }
  return color;
}
