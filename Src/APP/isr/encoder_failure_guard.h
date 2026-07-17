// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef ENCODER_FAILURE_GUARD_H
#define ENCODER_FAILURE_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_FAILURE_GUARD_LIMIT 3u

typedef struct {
  uint8_t failure_count;
} EncoderFailureGuardState;

bool EncoderFailureGuard_Record(EncoderFailureGuardState *state,
                                bool update_ok);
bool EncoderFailureGuard_ShouldFault(const EncoderFailureGuardState *state);
void EncoderFailureGuard_Reset(EncoderFailureGuardState *state);

#endif /* ENCODER_FAILURE_GUARD_H */