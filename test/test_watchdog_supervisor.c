// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "watchdog_supervisor.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static void test_feeds_only_after_a_healthy_window(void) {
  WatchdogSupervisorState state;
  WatchdogSupervisor_Init(&state, 100u, 20u, 100u, 200u);

  assert(WatchdogSupervisor_Evaluate(&state, 119u, 100u, 200u) ==
         WATCHDOG_WINDOW_PENDING);
  assert(WatchdogSupervisor_Evaluate(&state, 119u, 101u, 201u) ==
         WATCHDOG_WINDOW_PENDING);
  assert(WatchdogSupervisor_Evaluate(&state, 120u, 101u, 201u) ==
         WATCHDOG_WINDOW_HEALTHY);
}

static void test_heartbeat_cannot_be_reused_across_windows(void) {
  WatchdogSupervisorState state;
  WatchdogSupervisor_Init(&state, 0u, 20u, 10u, 20u);

  assert(WatchdogSupervisor_Evaluate(&state, 20u, 11u, 21u) ==
         WATCHDOG_WINDOW_HEALTHY);
  assert(WatchdogSupervisor_Evaluate(&state, 40u, 11u, 21u) ==
         WATCHDOG_WINDOW_UNHEALTHY);
  assert(WatchdogSupervisor_Evaluate(&state, 60u, 12u, 22u) ==
         WATCHDOG_WINDOW_HEALTHY);
}

static void test_tick_and_heartbeat_wrap_are_safe(void) {
  WatchdogSupervisorState state;
  WatchdogSupervisor_Init(&state, UINT32_MAX - 10u, 20u, UINT32_MAX,
                          UINT32_MAX);

  assert(WatchdogSupervisor_Evaluate(&state, 5u, 0u, 0u) ==
         WATCHDOG_WINDOW_PENDING);
  assert(WatchdogSupervisor_Evaluate(&state, 9u, 0u, 0u) ==
         WATCHDOG_WINDOW_HEALTHY);
}

static void test_requires_both_critical_execution_paths(void) {
  WatchdogSupervisorState state;
  WatchdogSupervisor_Init(&state, 0u, 20u, 1u, 1u);

  assert(WatchdogSupervisor_Evaluate(&state, 20u, 2u, 1u) ==
         WATCHDOG_WINDOW_UNHEALTHY);
  assert(WatchdogSupervisor_Evaluate(&state, 40u, 2u, 2u) ==
         WATCHDOG_WINDOW_UNHEALTHY);
  assert(WatchdogSupervisor_Evaluate(&state, 60u, 3u, 3u) ==
         WATCHDOG_WINDOW_HEALTHY);
}

int main(void) {
  test_feeds_only_after_a_healthy_window();
  test_heartbeat_cannot_be_reused_across_windows();
  test_tick_and_heartbeat_wrap_are_safe();
  test_requires_both_critical_execution_paths();
  puts("Watchdog supervisor tests: PASS");
  return 0;
}
