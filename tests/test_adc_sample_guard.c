// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "adc_sample_guard.h"

#include <assert.h>

static void accepts_complete_sample(void) {
  AdcSampleGuardState state = {0};
  const AdcSampleRaw sample = {.ia = 1000u, .ib = 1001u, .ic = 1002u,
                               .vbus = 2000u};

  assert(AdcSampleGuard_Check(&state, &sample, true, false) ==
         ADC_SAMPLE_GUARD_OK);
  assert(!AdcSampleGuard_ShouldFault(&state));
  assert(state.failure_count == 0u);
}

static void accepts_repeated_complete_samples(void) {
  AdcSampleGuardState state = {0};
  const AdcSampleRaw sample = {.ia = 1100u, .ib = 1101u, .ic = 1102u,
                               .vbus = 2200u};

  for (uint8_t i = 0u; i < ADC_SAMPLE_GUARD_FAILURE_LIMIT + 1u; ++i) {
    assert(AdcSampleGuard_Check(&state, &sample, true, false) ==
           ADC_SAMPLE_GUARD_OK);
  }
  assert(state.failure_count == 0u);
}

static void complete_sample_clears_failure_streak(void) {
  AdcSampleGuardState state = {0};
  const AdcSampleRaw sample = {.ia = 1200u, .ib = 1201u, .ic = 1202u,
                               .vbus = 2400u};

  assert(AdcSampleGuard_Check(&state, &sample, false, false) ==
         ADC_SAMPLE_GUARD_INCOMPLETE);
  assert(state.failure_count == 1u);
  assert(AdcSampleGuard_Check(&state, &sample, true, false) ==
         ADC_SAMPLE_GUARD_OK);
  assert(state.failure_count == 0u);
}

static void incomplete_and_adc_error_count_as_failures(void) {
  AdcSampleGuardState state = {0};
  const AdcSampleRaw sample = {.ia = 1300u, .ib = 1301u, .ic = 1302u,
                               .vbus = 2600u};

  assert(AdcSampleGuard_Check(&state, &sample, false, false) ==
         ADC_SAMPLE_GUARD_INCOMPLETE);
  assert(AdcSampleGuard_Check(&state, &sample, true, true) ==
         ADC_SAMPLE_GUARD_ADC_ERROR);
  assert(state.failure_count == 2u);
}

static void faults_at_limit_and_reset_clears_state(void) {
  AdcSampleGuardState state = {0};
  const AdcSampleRaw sample = {.ia = 1400u, .ib = 1401u, .ic = 1402u,
                               .vbus = 2800u};

  for (uint8_t i = 0u; i < ADC_SAMPLE_GUARD_FAILURE_LIMIT; ++i) {
    assert(AdcSampleGuard_Check(&state, &sample, false, false) ==
           ADC_SAMPLE_GUARD_INCOMPLETE);
  }
  assert(AdcSampleGuard_ShouldFault(&state));
  AdcSampleGuard_Reset(&state);
  assert(state.failure_count == 0u);
  assert(!AdcSampleGuard_ShouldFault(&state));
}

int main(void) {
  accepts_complete_sample();
  accepts_repeated_complete_samples();
  complete_sample_clears_failure_streak();
  incomplete_and_adc_error_count_as_failures();
  faults_at_limit_and_reset_clears_state();
  return 0;
}
