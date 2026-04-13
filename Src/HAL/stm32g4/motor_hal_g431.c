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

#ifndef BOARD_XSTAR
#include "motor_adc.h"
#include "config.h"
#include "board_config.h"
#include "motor_hal_api.h"
#include "hal_abstraction.h" // For HAL_GetTemperature()
#include <math.h>
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#include "tmr3109_encoder.h"
extern TMR3109_Handle_t tmr3109_encoder_data;
#else
#include "mt6816_encoder.h"
extern MT6816_Handle_t encoder_data;
#endif
extern CURRENT_DATA current_data;
/* ============================================================================
 * PWM Interface Implementation
 * ============================================================================
 */
static void G431_PWM_SetDuty(float dtc_a, float dtc_b, float dtc_c) {
  /* Ta→U(CH1), Tb→V(CH2), Tc→W(CH3) — must match ADC mapping:
   * Ia=JDR3(PA2)=U-phase, Ib=JDR2(PA1)=V-phase, Ic=JDR1(PA0)=W-phase.
   * Clarke: Ialpha=Ia(U-phase) → Ta must drive the same physical winding. */
  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&HW_PWM_TIMER);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_U, (uint16_t)(dtc_a * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_V, (uint16_t)(dtc_b * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_W, (uint16_t)(dtc_c * arr));
}
static void G431_PWM_Enable(void) {
  HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_U);
  HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_V);
  HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_W);
  HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, HW_PWM_CH_U);
  HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, HW_PWM_CH_V);
  HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, HW_PWM_CH_W);
}
static void G431_PWM_Disable(void) {
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_U);
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_V);
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_W);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_U);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_V);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_W);
}
static void G431_PWM_Brake(void) {
  // Low side ON, High side OFF
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_U, 0);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_V, 0);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_W, 0);
  // Ensure outputs are enabled
  G431_PWM_Enable();
}
static const Motor_HAL_PwmInterface_t g431_pwm = {.set_duty = G431_PWM_SetDuty,
                                                  .enable = G431_PWM_Enable,
                                                  .disable = G431_PWM_Disable,
                                                  .brake = G431_PWM_Brake};
/* ============================================================================
 * ADC Interface Implementation
 * ============================================================================
 */
// temperatureparam
#define TEMP_UPDATE_INTERVAL_MS 20    // 50Hz temperatureupdatefrequency
#define TEMP_LPF_ALPHA 0.1f           // filter (0-1，)
#define TEMP_ADC_MIN_VALID 100        //  ADC （）
#define TEMP_ADC_MAX_VALID 4000       //  ADC （）
#define TEMP_DEFAULT 25.0f            // temperature（fault）
static uint32_t s_last_temp_update = 0;
static float s_temp_filtered = TEMP_DEFAULT;
static bool s_temp_sensor_ok = true;
/**
 * @brief temperature（、filtererrorcheck）
 * @return temperature
 */
static float G431_ReadTemperature(void) {
  // 1. ：checkupdate
  uint32_t now = HAL_GetSystemTick();
  if (now - s_last_temp_update < TEMP_UPDATE_INTERVAL_MS) {
    return s_temp_filtered;  // filter
  }
  s_last_temp_update = now;
  // 2.  ADC （，filter）
  // ： DMA
  uint16_t adc_raw = adc2_dma_value[0][adc2_ch12];
  // 3. errorcheck：ADC
  if (adc_raw < TEMP_ADC_MIN_VALID || adc_raw > TEMP_ADC_MAX_VALID) {
    if (s_temp_sensor_ok) {
      // fault，error
      s_temp_sensor_ok = false;
      // ：ERROR_REPORT(ERROR_SENSOR_TEMP, "Temp sensor out of range");
    }
    // filter，update
    return s_temp_filtered;
  }
  // 4. temperature
  float temp;
  GetTempNtc(adc_raw, &temp);
  // 5. filter
  s_temp_filtered += TEMP_LPF_ALPHA * (temp - s_temp_filtered);
  s_temp_sensor_ok = true;
  return s_temp_filtered;
}
static void G431_ADC_Update(Motor_HAL_SensorData_t *data) {
  // Read from JDR registers (Injected group)
  // Note: This matches GetMotorADC1PhaseCurrent implementation
  // Scale factors from foc_driver.h
  // #define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES *
  // CURRENT_AMP_GAIN)) #define VOLTAGE_TO_ADC_FACTOR ...
  // We reuse the macros or constants if available, or redefine them
  // here/include them Ideally we should get them from a unified config. For
  // now, we assume macros exist or we access current_data
  // Let's use the existing current_data struct to hold offsets,
  // but do the reading here to fulfill the HAL contract.
  float adc_i_a = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IA;
  float adc_i_b = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IB;
  float adc_i_c = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IC;
  float adc_vbus = (float)HW_ADC_CURRENT.Instance->HW_ADC_JDR_VBUS;
  //  current_data (adc.h)，calibration ADC_SetCurrentOffsets
  data->i_a = (adc_i_a - current_data.Ia_offset) * FAC_CURRENT;
  data->i_b = (adc_i_b - current_data.Ib_offset) * FAC_CURRENT;
  data->i_c = (adc_i_c - current_data.Ic_offset) * FAC_CURRENT;
  data->v_bus = adc_vbus * VOLTAGE_TO_ADC_FACTOR;
  // temperature（、filtererrorcheck）
  data->temp = G431_ReadTemperature();
}
static void G431_ADC_CalibrateOffsets(void) {
  /* Wait for JEOS (injected end-of-sequence) per sample instead of HAL_Delay.
   * JDR registers are read-only and hold the last value until next conversion,
   * so concurrent reads from the ADC ISR and this function are safe. */
  uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    /* Spin until JEOS set, or 10 ms timeout (ADC runs at 20 kHz → fires ~50 µs) */
    uint32_t t0 = HAL_GetTick();
    while (!(HW_ADC_CURRENT.Instance->ISR & ADC_ISR_JEOS)) {
      if (HAL_GetTick() - t0 > 10U) break;
    }
    HW_ADC_CURRENT.Instance->ISR = ADC_ISR_JEOS; /* W1C — clear flag */
    sum_a += HW_ADC_CURRENT.Instance->HW_ADC_JDR_IA;
    sum_b += HW_ADC_CURRENT.Instance->HW_ADC_JDR_IB;
    sum_c += HW_ADC_CURRENT.Instance->HW_ADC_JDR_IC;
  }
  current_data.Ia_offset = (float)sum_a / samples;
  current_data.Ib_offset = (float)sum_b / samples;
  current_data.Ic_offset = (float)sum_c / samples;
}
static const Motor_HAL_AdcInterface_t g431_adc = {
    .update = G431_ADC_Update, .calibrate_offsets = G431_ADC_CalibrateOffsets};
/* ============================================================================
 * Encoder Interface Implementation
 * ============================================================================
 */
static void G431_Encoder_Update(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  TMR3109_Update(&tmr3109_encoder_data, CURRENT_MEASURE_PERIOD);
#else
  MT6816_Update(&encoder_data, CURRENT_MEASURE_PERIOD);
#endif
}
static void G431_Encoder_GetData(Motor_HAL_EncoderData_t *data) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  data->angle_rad    = tmr3109_encoder_data.mec_angle_rad;
  data->velocity_rad = tmr3109_encoder_data.velocity_rad_s;
  data->elec_angle   = tmr3109_encoder_data.elec_angle_rad;
  data->raw_value    = (int32_t)tmr3109_encoder_data.raw_angle;
#else
  data->angle_rad    = encoder_data.mec_angle_rad;
  data->velocity_rad = encoder_data.velocity_rad_s;
  data->elec_angle   = encoder_data.elec_angle_rad;
  data->raw_value    = encoder_data.raw_angle;
#endif
}
static void G431_Encoder_SetOffset(float offset) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  tmr3109_encoder_data.offset_rev = offset;
#else
  encoder_data.offset_rev = offset;
#endif
}
static const Motor_HAL_EncoderInterface_t g431_encoder = {
    .update    = G431_Encoder_Update,
    .get_data  = G431_Encoder_GetData,
    .set_offset = G431_Encoder_SetOffset};
/* ============================================================================
 * Main Handle Construction
 * ============================================================================
 */
Motor_HAL_Handle_t g431_hal_handle = {
    .pwm = &g431_pwm, .adc = &g431_adc, .encoder = &g431_encoder};

#endif /* BOARD_XSTAR */
