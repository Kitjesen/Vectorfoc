// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "encoder_failure_guard.h"

#include <assert.h>

static void test_faults_on_third_consecutive_failure(void) {
  EncoderFailureGuardState state = {0};

  assert(!EncoderFailureGuard_Record(&state, false));
  assert(state.failure_count == 1u);
  assert(!EncoderFailureGuard_Record(&state, false));
  assert(state.failure_count == 2u);
  assert(EncoderFailureGuard_Record(&state, false));
  assert(state.failure_count == ENCODER_FAILURE_GUARD_LIMIT);
}

static void test_success_clears_failure_streak(void) {
  EncoderFailureGuardState state = {0};

  assert(!EncoderFailureGuard_Record(&state, false));
  assert(!EncoderFailureGuard_Record(&state, false));
  assert(state.failure_count == 2u);
  assert(!EncoderFailureGuard_Record(&state, true));
  assert(state.failure_count == 0u);
  assert(!EncoderFailureGuard_Record(&state, false));
  assert(state.failure_count == 1u);
}

static void test_reset_clears_state(void) {
  EncoderFailureGuardState state = {0};

  assert(EncoderFailureGuard_Record(&state, false) == false);
  EncoderFailureGuard_Reset(&state);
  assert(state.failure_count == 0u);
}

int main(void) {
  test_faults_on_third_consecutive_failure();
  test_success_clears_failure_streak();
  test_reset_clears_state();
  return 0;
}