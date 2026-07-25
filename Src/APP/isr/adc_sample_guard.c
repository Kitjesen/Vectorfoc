// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "adc_sample_guard.h"
#include <stddef.h>

static void AdcSampleGuard_RecordFailure(AdcSampleGuardState *state) {
  if (state->failure_count < UINT8_MAX) {
    state->failure_count++;
  }
}

AdcSampleGuardStatus AdcSampleGuard_Check(AdcSampleGuardState *state,
                                          const AdcSampleRaw *sample,
                                          bool sequence_complete,
                                          bool adc_error) {
  if (state == NULL || sample == NULL) {
    return ADC_SAMPLE_GUARD_ADC_ERROR;
  }

  if (!sequence_complete) {
    AdcSampleGuard_RecordFailure(state);
    return ADC_SAMPLE_GUARD_INCOMPLETE;
  }

  if (adc_error) {
    AdcSampleGuard_RecordFailure(state);
    return ADC_SAMPLE_GUARD_ADC_ERROR;
  }

  state->failure_count = 0;
  return ADC_SAMPLE_GUARD_OK;
}

bool AdcSampleGuard_ShouldFault(const AdcSampleGuardState *state) {
  return state != NULL &&
         state->failure_count >= ADC_SAMPLE_GUARD_FAILURE_LIMIT;
}

void AdcSampleGuard_Reset(AdcSampleGuardState *state) {
  if (state == NULL) {
    return;
  }
  state->failure_count = 0U;
}
