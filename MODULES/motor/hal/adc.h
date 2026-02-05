/**
 * @file    adc.h
 * @brief   ADC driver for motor current sensing and temperature monitoring.
 * @details
 * - Context: Low-level driver, handles raw ADC data conversion and filtering.
 * - Units:   Temperature [degC], Current offsets [LSB].
 * - Note:    Relies on 'bsp_adc.h' for hardware definitions.
 */

#ifndef CORE_ADC_H
#define CORE_ADC_H

#include "bsp_adc.h"
#include "common.h"
#include "main.h"

/**
 * @brief Initialize the ADC driver with a specific hardware handle.
 * @param hadc Pointer to the ADC handle (e.g. &hadc1).
 */
void ADC_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief Runtime ADC data and offset calibration.
 */
typedef struct {
  ADC_HandleTypeDef *hadc;    /**< ADC Handle */
  float Temp_Result;          /**< [degC] Board temperature */
  float Ia_offset;            /**< [LSB] Phase A current offset */
  float Ib_offset;            /**< [LSB] Phase B current offset */
  float Ic_offset;            /**< [LSB] Phase C current offset */
  float current_offset_sum_a; /**< [LSB*samples] Accumulator for offset A */
  float current_offset_sum_b; /**< [LSB*samples] Accumulator for offset B */
  float current_offset_sum_c; /**< [LSB*samples] Accumulator for offset C */
} CURRENT_DATA;

extern CURRENT_DATA current_data;

/**
 * @brief 设置电流采样零偏 (LSB)，供校准完成后写入
 * @param Ia Phase A 零偏 [LSB]
 * @param Ib Phase B 零偏 [LSB]
 * @param Ic Phase C 零偏 [LSB]
 */
void ADC_SetCurrentOffsets(float Ia, float Ib, float Ic);

/**
 * @brief ADC1 channel mapping.
 */
typedef enum {
  adc1_ch1 = 0,
  adc1_ch2 = 1,
  adc1_ch3 = 2,
  adc1_ch4 = 3
} adc1_num;

/**
 * @brief ADC2 channel mapping.
 */
typedef enum {
  adc2_ch12 = 0,
} adc2_num;

/**
 * @brief  Convert NTC ADC raw value to temperature.
 * @param  value_adc  Raw ADC reading.
 * @param  value_temp [out] Pointer to store result [degC].
 */
void GetTempNtc(uint16_t value_adc, float *value_temp);

/**
 * @brief  Apply median filter to ADC1 channel (in-place sort).
 * @param  channel ADC1 channel index.
 * @return Filtered value [LSB].
 */
uint16_t adc1_median_filter(uint8_t channel);

/**
 * @brief  Apply average filter to ADC1 channel.
 * @param  channel ADC1 channel index.
 * @return Filtered value [LSB].
 */
uint16_t adc1_avg_filter(uint8_t channel);

/**
 * @brief  Apply median filter to ADC2 channel (in-place sort).
 * @param  channel ADC2 channel index.
 * @return Filtered value [LSB].
 */
uint16_t adc2_median_filter(uint8_t channel);

/**
 * @brief  Apply average filter to ADC2 channel.
 * @param  channel ADC2 channel index.
 * @return Filtered value [LSB].
 */
uint16_t adc2_avg_filter(uint8_t channel);

#endif // CORE_ADC_H
