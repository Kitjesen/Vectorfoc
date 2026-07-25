// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "motor_status_led.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static void expect_idle_blinks_at_one_hz(void) {
  MotorStatusLedState led;
  MotorStatusLed_Init(&led);

  for (uint32_t i = 0u; i < 100u; ++i) {
    assert(MotorStatusLed_Update(&led, STATE_MODE_IDLE, false) ==
           MOTOR_STATUS_LED_COLOR_PURPLE);
  }
  for (uint32_t i = 0u; i < 100u; ++i) {
    assert(MotorStatusLed_Update(&led, STATE_MODE_IDLE, false) ==
           MOTOR_STATUS_LED_COLOR_BLACK);
  }
  assert(MotorStatusLed_Update(&led, STATE_MODE_IDLE, false) ==
         MOTOR_STATUS_LED_COLOR_PURPLE);
}

static void expect_detecting_uses_idle_blink(void) {
  MotorStatusLedState led;
  MotorStatusLed_Init(&led);

  assert(MotorStatusLed_Update(&led, STATE_MODE_DETECTING, false) ==
         MOTOR_STATUS_LED_COLOR_PURPLE);
}

static void expect_fault_and_running_colors(void) {
  MotorStatusLedState led;
  MotorStatusLed_Init(&led);

  assert(MotorStatusLed_Update(&led, STATE_MODE_GUARD, true) ==
         MOTOR_STATUS_LED_COLOR_RED);
  assert(MotorStatusLed_Update(&led, STATE_MODE_RUNNING, false) ==
         MOTOR_STATUS_LED_COLOR_CYAN);
}

static void expect_guard_blinks_at_100ms(void) {
  MotorStatusLedState led;
  MotorStatusLed_Init(&led);

  for (uint32_t i = 0u; i < 10u; ++i) {
    assert(MotorStatusLed_Update(&led, STATE_MODE_GUARD, false) ==
           MOTOR_STATUS_LED_COLOR_RED);
  }
  for (uint32_t i = 0u; i < 10u; ++i) {
    assert(MotorStatusLed_Update(&led, STATE_MODE_GUARD, false) ==
           MOTOR_STATUS_LED_COLOR_BLACK);
  }
  assert(MotorStatusLed_Update(&led, STATE_MODE_GUARD, false) ==
         MOTOR_STATUS_LED_COLOR_RED);
}

int main(void) {
  expect_idle_blinks_at_one_hz();
  expect_detecting_uses_idle_blink();
  expect_fault_and_running_colors();
  expect_guard_blinks_at_100ms();

  puts("Motor status LED tests: PASS");
  return 0;
}
