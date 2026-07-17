// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "encoder_failure_guard.h"

#include <string.h>

bool EncoderFailureGuard_Record(EncoderFailureGuardState *state,
                                bool update_ok) {
  if (state == NULL) {
    return false;
  }
  if (update_ok) {
    state->failure_count = 0u;
    return false;
  }
  if (state->failure_count < UINT8_MAX) {
    state->failure_count++;
  }
  return EncoderFailureGuard_ShouldFault(state);
}

bool EncoderFailureGuard_ShouldFault(const EncoderFailureGuardState *state) {
  return state != NULL && state->failure_count >= ENCODER_FAILURE_GUARD_LIMIT;
}

void EncoderFailureGuard_Reset(EncoderFailureGuardState *state) {
  if (state == NULL) {
    return;
  }
  memset(state, 0, sizeof(*state));
}