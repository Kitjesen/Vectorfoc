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
 * @file hal_adc.c
 * @brief ADC ?
 * @note  HAL  motor_data.components.hal->adc
 *       motor_data 
 *       ㄦ?header €?(HAL_ADC_*)
 */
#include "hal_adc.h"
#include "motor.h"
/* € ADC  */
static Motor_HAL_SensorData_t s_adc_cache = {0};
/**
 * @brief ㄥ ADC ュ (?
 * @note  HAL ュ?
 */
int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface) {
  (void)interface; // ?
  return 0;        //
}
int MHAL_ADC_Init(void) {
  // ADC  MX_ADC1_Init/MX_ADC2_Init ?main.c ?
  return 0;
}
int MHAL_ADC_Start(void) {
  // ADC ?adc_bsp_init()
  return 0;
}
int MHAL_ADC_Stop(void) {
  // ?
  return 0;
}
int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->update == NULL)
    return -1;
  //  ADC
  motor_data.components.hal->adc->update(&s_adc_cache);
  if (Ia)
    *Ia = s_adc_cache.i_a;
  if (Ib)
    *Ib = s_adc_cache.i_b;
  if (Ic)
    *Ic = s_adc_cache.i_c;
  return 0;
}
float MHAL_ADC_GetVbus(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->update == NULL)
    return 0.0f;
  motor_data.components.hal->adc->update(&s_adc_cache);
  return s_adc_cache.v_bus;
}
float MHAL_ADC_GetTemperature(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->update == NULL)
    return 0.0f;
  motor_data.components.hal->adc->update(&s_adc_cache);
  return s_adc_cache.temp;
}
int MHAL_ADC_CalibrateCurrent(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->calibrate_offsets == NULL)
    return -1;
  motor_data.components.hal->adc->calibrate_offsets();
  return 0;
}
