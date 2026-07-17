// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "watchdog_supervisor.h"

#include <stddef.h>

static volatile uint32_t s_foc_heartbeat;
static volatile uint32_t s_comm_heartbeat;

void WatchdogSupervisor_Init(WatchdogSupervisorState *state, uint32_t now_ms,
                             uint32_t window_ms, uint32_t foc_heartbeat,
                             uint32_t comm_heartbeat) {
  if (state == NULL) {
    return;
  }

  state->window_start_ms = now_ms;
  state->window_ms = window_ms;
  state->last_foc_heartbeat = foc_heartbeat;
  state->last_comm_heartbeat = comm_heartbeat;
  state->observed_foc_heartbeat = false;
  state->observed_comm_heartbeat = false;
}

WatchdogWindowStatus WatchdogSupervisor_Evaluate(WatchdogSupervisorState *state,
                                                 uint32_t now_ms,
                                                 uint32_t foc_heartbeat,
                                                 uint32_t comm_heartbeat) {
  if (state == NULL || state->window_ms == 0u) {
    return WATCHDOG_WINDOW_UNHEALTHY;
  }

  if (foc_heartbeat != state->last_foc_heartbeat) {
    state->last_foc_heartbeat = foc_heartbeat;
    state->observed_foc_heartbeat = true;
  }
  if (comm_heartbeat != state->last_comm_heartbeat) {
    state->last_comm_heartbeat = comm_heartbeat;
    state->observed_comm_heartbeat = true;
  }

  if ((uint32_t)(now_ms - state->window_start_ms) < state->window_ms) {
    return WATCHDOG_WINDOW_PENDING;
  }

  WatchdogWindowStatus status =
      state->observed_foc_heartbeat && state->observed_comm_heartbeat
          ? WATCHDOG_WINDOW_HEALTHY
          : WATCHDOG_WINDOW_UNHEALTHY;
  state->window_start_ms = now_ms;
  state->observed_foc_heartbeat = false;
  state->observed_comm_heartbeat = false;
  return status;
}

void WatchdogSupervisor_MarkFOC(void) { ++s_foc_heartbeat; }

uint32_t WatchdogSupervisor_GetFOCHeartbeat(void) { return s_foc_heartbeat; }

void WatchdogSupervisor_MarkComm(void) { ++s_comm_heartbeat; }

uint32_t WatchdogSupervisor_GetCommHeartbeat(void) { return s_comm_heartbeat; }
