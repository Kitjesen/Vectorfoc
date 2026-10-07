// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file    adc.h
 * @brief   ADC driver for motor current sensing and temperature monitoring.
 * @details
 * - Context: Low-level driver, handles raw ADC data conversion and filtering.
 * - Units:   Temperature [degC], Current offsets [LSB].
 * - Note:    Relies on 'board_adc.h' for hardware definitions.
 */
#ifndef CORE_ADC_H
#define CORE_ADC_H
#include "board_adc.h"
#include "common.h"
#include "main.h"
typedef struct {
  float i_a, i_b, i_c; /* A */
  float v_bus;        /* V */
  float temp;         /* degrees C */
} MotorSensorData;
void ADC_ReadSensors(MotorSensorData *data);
/**
 * @brief Runtime ADC data and offset calibration.
 */
typedef struct {
  float Ia_offset; /**< [LSB] Phase A current offset */
  float Ib_offset; /**< [LSB] Phase B current offset */
  float Ic_offset; /**< [LSB] Phase C current offset */
} CURRENT_DATA;
extern CURRENT_DATA current_data;
/**
 * @brief setcurrentsample (LSB)，calibrationdone
 * @param Ia Phase A  [LSB]
 * @param Ib Phase B  [LSB]
 * @param Ic Phase C  [LSB]
 */
void ADC_SetCurrentOffsets(float Ia, float Ib, float Ic);
/**
 * @brief  Convert NTC ADC raw value to temperature.
 * @param  value_adc  Raw ADC reading.
 * @param  value_temp [out] Pointer to store result [degC].
 */
void GetTempNtc(uint16_t value_adc, float *value_temp);
/* Filters copy DMA data before sorting; the DMA-owned buffers are never
 * reordered in place.  Invalid channels fail closed with zero. */
uint16_t adc1_median_filter(uint8_t channel);
uint16_t adc1_avg_filter(uint8_t channel);
uint16_t adc2_median_filter(uint8_t channel);
uint16_t adc2_avg_filter(uint8_t channel);
#endif // CORE_ADC_H
