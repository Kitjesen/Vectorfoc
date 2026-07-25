// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ADC_SAMPLE_GUARD_H
#define ADC_SAMPLE_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define ADC_SAMPLE_GUARD_FAILURE_LIMIT 4u

typedef struct {
  uint16_t ia;
  uint16_t ib;
  uint16_t ic;
  uint16_t vbus;
} AdcSampleRaw;

typedef enum {
  ADC_SAMPLE_GUARD_OK = 0,
  ADC_SAMPLE_GUARD_INCOMPLETE,
  ADC_SAMPLE_GUARD_ADC_ERROR,
} AdcSampleGuardStatus;

typedef struct {
  uint8_t failure_count;
} AdcSampleGuardState;

AdcSampleGuardStatus AdcSampleGuard_Check(AdcSampleGuardState *state,
                                          const AdcSampleRaw *sample,
                                          bool sequence_complete,
                                          bool adc_error);

bool AdcSampleGuard_ShouldFault(const AdcSampleGuardState *state);
void AdcSampleGuard_Reset(AdcSampleGuardState *state);

#endif /* ADC_SAMPLE_GUARD_H */