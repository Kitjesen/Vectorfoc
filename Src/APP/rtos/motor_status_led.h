// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOR_STATUS_LED_H
#define MOTOR_STATUS_LED_H

#include "motor.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_STATUS_LED_COLOR_RED 0u
#define MOTOR_STATUS_LED_COLOR_CYAN 3u
#define MOTOR_STATUS_LED_COLOR_BLACK 7u
#define MOTOR_STATUS_LED_COLOR_PURPLE 9u

typedef struct {
  uint32_t idle_tick;
  uint32_t guard_tick;
} MotorStatusLedState;

void MotorStatusLed_Init(MotorStatusLedState *led);
uint8_t MotorStatusLed_Update(MotorStatusLedState *led, STATE_MODE mode,
                              bool has_active_fault);

#endif /* MOTOR_STATUS_LED_H */
