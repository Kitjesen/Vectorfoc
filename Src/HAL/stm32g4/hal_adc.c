// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file hal_adc.c
 * @brief Typed adapter from the legacy MHAL API to a replaceable motor ADC
 * port.
 */
#include "hal_adc.h"

#include <stddef.h>

static const Motor_HAL_AdcInterface_t *s_adc;
static Motor_HAL_SensorData_t s_adc_cache;

int MHAL_ADC_Bind(const Motor_HAL_AdcInterface_t *interface) {
  s_adc = interface;
  return interface != NULL ? 0 : -1;
}

int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface) {
  (void)interface;
  return -1;
}

int MHAL_ADC_Init(void) { return s_adc != NULL ? 0 : -1; }
int MHAL_ADC_Start(void) { return s_adc != NULL ? 0 : -1; }
int MHAL_ADC_Stop(void) { return s_adc != NULL ? 0 : -1; }

static int MHAL_ADC_Refresh(void) {
  if (s_adc == NULL || s_adc->update == NULL) {
    return -1;
  }
  s_adc->update(&s_adc_cache);
  return 0;
}

int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic) {
  if (MHAL_ADC_Refresh() != 0) {
    return -1;
  }
  if (Ia != NULL) {
    *Ia = s_adc_cache.i_a;
  }
  if (Ib != NULL) {
    *Ib = s_adc_cache.i_b;
  }
  if (Ic != NULL) {
    *Ic = s_adc_cache.i_c;
  }
  return 0;
}

float MHAL_ADC_GetVbus(void) {
  return MHAL_ADC_Refresh() == 0 ? s_adc_cache.v_bus : 0.0f;
}

float MHAL_ADC_GetTemperature(void) {
  return MHAL_ADC_Refresh() == 0 ? s_adc_cache.temp : 0.0f;
}

int MHAL_ADC_CalibrateCurrent(void) {
  if (s_adc == NULL || s_adc->calibrate_offsets == NULL) {
    return -1;
  }
  return s_adc->calibrate_offsets() ? 0 : -1;
}
