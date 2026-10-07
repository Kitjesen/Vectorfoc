// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef APP_H
#define APP_H
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Prepare software and calibrated ADCs, then start sampling with power off.
 * Passive current-offset averaging finishes in the ADC ISR; a fresh explicit
 * enable command must pass the safety gate before any phase output starts. */
void App_Init(void);
bool App_IsFocRuntimeReady(void);

void StartDefaultTask(void const *argument); /* USB: 1 ms */
void StartGuardTask(void const *argument);   /* Protection: 5 ms */
void StartCustomTask(void const *argument);  /* CAN: 2 ms */

#ifdef __cplusplus
}
#endif
#endif
