// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef COMM_TEST_CMSIS_OS_H
#define COMM_TEST_CMSIS_OS_H
#include <stdint.h>
typedef enum { osOK = 0 } osStatus;
osStatus osDelay(uint32_t millisec);
#endif
