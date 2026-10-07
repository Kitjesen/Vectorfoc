// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WATCHDOG_SUPERVISOR_H
#define WATCHDOG_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#ifndef WATCHDOG_SUPERVISION_WINDOW_MS
#define WATCHDOG_SUPERVISION_WINDOW_MS 20u
#endif

typedef enum {
  WATCHDOG_WINDOW_PENDING = 0,
  WATCHDOG_WINDOW_HEALTHY,
  WATCHDOG_WINDOW_UNHEALTHY,
} WatchdogWindowStatus;

typedef struct {
  uint32_t window_start_ms;
  uint32_t window_ms;
  uint32_t last_foc_heartbeat;
  uint32_t last_comm_heartbeat;
  bool observed_foc_heartbeat;
  bool observed_comm_heartbeat;
} WatchdogSupervisorState;

void WatchdogSupervisor_Init(WatchdogSupervisorState *state, uint32_t now_ms,
                             uint32_t window_ms, uint32_t foc_heartbeat,
                             uint32_t comm_heartbeat);
WatchdogWindowStatus WatchdogSupervisor_Evaluate(WatchdogSupervisorState *state,
                                                 uint32_t now_ms,
                                                 uint32_t foc_heartbeat,
                                                 uint32_t comm_heartbeat);
void WatchdogSupervisor_MarkFOC(void);
uint32_t WatchdogSupervisor_GetFOCHeartbeat(void);
void WatchdogSupervisor_MarkComm(void);
uint32_t WatchdogSupervisor_GetCommHeartbeat(void);

#endif
