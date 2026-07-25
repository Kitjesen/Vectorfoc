// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef TEST_MOTOR_HAL_G431_MOTOR_ADC_H
#define TEST_MOTOR_HAL_G431_MOTOR_ADC_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "board_config.h"

typedef struct {
  ADC_HandleTypeDef *hadc;
  float Temp_Result;
  float Ia_offset;
  float Ib_offset;
  float Ic_offset;
  float current_offset_sum_a;
  float current_offset_sum_b;
  float current_offset_sum_c;
} CURRENT_DATA;

typedef enum {
  adc2_ch12 = 0,
} adc2_num;

typedef struct {
  uint16_t phase_a;
  uint16_t phase_b;
  uint16_t phase_c;
} MotorAdcCurrentSample;

typedef struct {
  float phase_a;
  float phase_b;
  float phase_c;
} MotorAdcCurrentOffsets;

typedef bool (*MotorAdcSampleReader)(void *context,
                                     MotorAdcCurrentSample *sample);

extern CURRENT_DATA current_data;
extern uint16_t adc2_dma_value[1][1];

#ifndef FAC_CURRENT
#define FAC_CURRENT HW_FAC_CURRENT
#endif
#ifndef VOLTAGE_TO_ADC_FACTOR
#define VOLTAGE_TO_ADC_FACTOR HW_VOLTAGE_FACTOR
#endif

bool MotorAdc_CalibrateOffsets(MotorAdcSampleReader read_next, void *context,
                               uint32_t sample_count, uint16_t valid_min,
                               uint16_t valid_max,
                               MotorAdcCurrentOffsets *result);

#endif