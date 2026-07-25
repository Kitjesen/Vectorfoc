// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef TEST_MOTOR_HAL_G431_HAL_ABSTRACTION_H
#define TEST_MOTOR_HAL_G431_HAL_ABSTRACTION_H

#include <stdint.h>

uint32_t HAL_GetSystemTick(void);
void HAL_WatchdogFeed(void);

#endif