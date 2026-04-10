// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/**
 * @file motor_adc.h (TEST_ENV stub)
 */
#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float Ia_offset;
    float Ib_offset;
    float Ic_offset;
} CURRENT_DATA;

extern CURRENT_DATA current_data;

static inline void ADC_SetCurrentOffsets(float a, float b, float c) {
    (void)a; (void)b; (void)c;
}
static inline bool ADC_GetCurrentOffsets(float *a, float *b, float *c) {
    if (a) *a = 0; if (b) *b = 0; if (c) *c = 0;
    return true;
}

#endif /* MOTOR_ADC_H */
