// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "adc_sample_guard.h"

#include <assert.h>

static void test_accepts_first_complete_sample(void) {
  AdcSampleGuardState state = {0};
  AdcSampleRaw sample = {.ia = 1000u, .ib = 1001u, .ic = 1002u, .vbus = 2000u};

  assert(AdcSampleGuard_Check(&state, &sample, true, false) ==
         ADC_SAMPLE_GUARD_OK);
  assert(!AdcSampleGuard_ShouldFault(&state));
  assert(state.failure_count == 0u);
}

static void test_accepts_legitimately_repeated_complete_samples(void) {
  AdcSampleGuardState state = {0};
  AdcSampleRaw sample = {.ia = 1100u, .ib = 1101u, .ic = 1102u, .vbus = 2200u};

  for (uint8_t i = 0u; i < ADC_SAMPLE_GUARD_FAILURE_LIMIT + 1u; ++i) {
    assert(AdcSampleGuard_Check(&state, &sample, true, false) ==
           ADC_SAMPLE_GUARD_OK);
    assert(!AdcSampleGuard_ShouldFault(&state));
  }
  assert(state.failure_count == 0u);
}

static void test_new_sample_clears_failure_streak(void) {
  AdcSampleGuardState state = {0};
  AdcSampleRaw first = {.ia = 1200u, .ib = 1201u, .ic = 1202u, .vbus = 2400u};
  AdcSampleRaw next = {.ia = 1201u, .ib = 1201u, .ic = 1202u, .vbus = 2400u};

  assert(AdcSampleGuard_Check(&state, &first, false, false) ==
         ADC_SAMPLE_GUARD_INCOMPLETE);
  assert(state.failure_count == 1u);

  assert(AdcSampleGuard_Check(&state, &next, true, false) ==
         ADC_SAMPLE_GUARD_OK);
  assert(state.failure_count == 0u);
  assert(!AdcSampleGuard_ShouldFault(&state));
}

static void test_incomplete_and_adc_error_count_as_failures(void) {
  AdcSampleGuardState state = {0};
  AdcSampleRaw sample = {.ia = 1300u, .ib = 1301u, .ic = 1302u, .vbus = 2600u};

  assert(AdcSampleGuard_Check(&state, &sample, false, false) ==
         ADC_SAMPLE_GUARD_INCOMPLETE);
  assert(state.failure_count == 1u);

  assert(AdcSampleGuard_Check(&state, &sample, true, true) ==
         ADC_SAMPLE_GUARD_ADC_ERROR);
  assert(state.failure_count == 2u);
}

static void test_reset_clears_state(void) {
  AdcSampleGuardState state = {0};
  AdcSampleRaw sample = {.ia = 1400u, .ib = 1401u, .ic = 1402u, .vbus = 2800u};

  for (uint8_t i = 0u; i < ADC_SAMPLE_GUARD_FAILURE_LIMIT; ++i) {
    assert(AdcSampleGuard_Check(&state, &sample, false, false) ==
           ADC_SAMPLE_GUARD_INCOMPLETE);
  }
  assert(AdcSampleGuard_ShouldFault(&state));

  AdcSampleGuard_Reset(&state);
  assert(state.failure_count == 0u);
  assert(!AdcSampleGuard_ShouldFault(&state));
  assert(AdcSampleGuard_Check(&state, &sample, true, false) ==
         ADC_SAMPLE_GUARD_OK);
}

int main(void) {
  test_accepts_first_complete_sample();
  test_accepts_legitimately_repeated_complete_samples();
  test_new_sample_clears_failure_streak();
  test_incomplete_and_adc_error_count_as_failures();
  test_reset_clears_state();
  return 0;
}