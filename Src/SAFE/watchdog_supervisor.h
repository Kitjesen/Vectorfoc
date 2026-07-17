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

#ifndef WATCHDOG_SUPERVISOR_H
#define WATCHDOG_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/**
 * @brief Initialize one watchdog health window.
 *
 * The caller owns the state.  This makes the decision logic deterministic and
 * host-testable; the hardware watchdog is fed separately by the guard task.
 */
void WatchdogSupervisor_Init(WatchdogSupervisorState *state, uint32_t now_ms,
                             uint32_t window_ms, uint32_t foc_heartbeat,
                             uint32_t comm_heartbeat);

/**
 * @brief Evaluate whether the completed window observed fresh FOC activity.
 *
 * A heartbeat may satisfy exactly one completed window.  Unsigned subtraction
 * intentionally makes the millisecond tick safe across uint32_t wraparound.
 */
WatchdogWindowStatus WatchdogSupervisor_Evaluate(WatchdogSupervisorState *state,
                                                 uint32_t now_ms,
                                                 uint32_t foc_heartbeat,
                                                 uint32_t comm_heartbeat);

/**
 * @brief Mark completion of one FOC interrupt.
 *
 * This function has a single writer (the ADC ISR).  The guard task only reads
 * the aligned 32-bit counter, avoiding a shared read-modify-write bitmask.
 */
void WatchdogSupervisor_MarkFOC(void);

/** @brief Return the current FOC heartbeat sequence. */
uint32_t WatchdogSupervisor_GetFOCHeartbeat(void);

/** @brief Mark completion of one communication-service iteration. */
void WatchdogSupervisor_MarkComm(void);

/** @brief Return the current communication heartbeat sequence. */
uint32_t WatchdogSupervisor_GetCommHeartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_SUPERVISOR_H */
