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
#include "board_config.h"
#include "bsp_dwt.h"
#include "config.h"
#include "hal_abstraction.h" // For HAL_GetTemperature()
#include "motor_adc.h"
#include "motor_hal_api.h"
#include "position_sensor_motor_hal.h"
#include <math.h>
extern CURRENT_DATA current_data;
/* ============================================================================
 * PWM Interface Implementation
 * ============================================================================
 */
static void G431_PWM_SetDuty(float dtc_a, float dtc_b, float dtc_c) {
  //  TIM register， inner.c done
  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&HW_PWM_TIMER);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A,
                        (uint16_t)(dtc_a * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B,
                        (uint16_t)(dtc_b * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C,
                        (uint16_t)(dtc_c * arr));
}
static void G431_PWM_Disable(void);
static bool G431_PWM_StartSampling(void) {
  G431_PWM_Disable();
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A, 0U);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B, 0U);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C, 0U);
  return HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_TRIG) == HAL_OK;
}
static bool G431_PWM_StartChannel(uint32_t channel) {
  return HAL_TIM_PWM_Start(&HW_PWM_TIMER, channel) == HAL_OK &&
         HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, channel) == HAL_OK;
}
static bool G431_PWM_Enable(void) {
  if (!G431_PWM_StartChannel(HW_PWM_CH_PHASE_A) ||
      !G431_PWM_StartChannel(HW_PWM_CH_PHASE_B) ||
      !G431_PWM_StartChannel(HW_PWM_CH_PHASE_C)) {
    G431_PWM_Disable();
    return false;
  }
  return true;
}
static void G431_PWM_Disable(void) {
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A);
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B);
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C);
}
static void G431_PWM_Brake(void) {
  /* Brake is a fail-safe zero-output request, not an implicit enable. */
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A, 0);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B, 0);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C, 0);
  G431_PWM_Disable();
}
static const Motor_HAL_PwmInterface_t g431_pwm = {.set_duty = G431_PWM_SetDuty,
                                                  .start_sampling =
                                                      G431_PWM_StartSampling,
                                                  .enable = G431_PWM_Enable,
                                                  .disable = G431_PWM_Disable,
                                                  .brake = G431_PWM_Brake};
/* ============================================================================
 * ADC Interface Implementation
 * ============================================================================
 */
// temperatureparam
#define TEMP_UPDATE_INTERVAL_MS 20 // 50Hz temperatureupdatefrequency
#define TEMP_LPF_ALPHA 0.1f        // filter (0-1，)
#define TEMP_ADC_MIN_VALID 100     //  ADC （）
#define TEMP_ADC_MAX_VALID 4000    //  ADC （）
#define TEMP_DEFAULT 25.0f         // temperature（fault）
static uint32_t s_last_temp_update = 0;
static float s_temp_filtered = TEMP_DEFAULT;
static bool s_temp_sensor_ok = true;

static float G431_NTC_ConvertToTemp(uint16_t adc_raw) {
  if (adc_raw == 0U || adc_raw > TEMP_ADC_MAX_VALID) {
    return NAN;
  }
  /* VectorFOC topology: 3.3 V -> 10k pull-up -> ADC -> NTC -> GND. */
  float r_ntc =
      HW_NTC_PULLUP * (float)adc_raw / (float)(HW_ADC_RESOLUTION - adc_raw);
  float denominator = logf(r_ntc / HW_NTC_R25) + HW_NTC_B_VALUE / 298.15f;
  if (!isfinite(denominator) || denominator <= 0.0f) {
    return NAN;
  }
  return HW_NTC_B_VALUE / denominator - 273.15f;
}
/**
 * @brief temperature（、filtererrorcheck）
 * @return temperature
 */
static float G431_ReadTemperature(void) {
  // 1. ：checkupdate
  uint32_t now = HAL_GetSystemTick();
  if (now - s_last_temp_update < TEMP_UPDATE_INTERVAL_MS) {
    return s_temp_filtered; // filter
  }
  s_last_temp_update = now;
  // 2.  ADC （，filter）
  // ： DMA
  uint16_t adc_raw = adc2_dma_value[0][adc2_ch12];
  // 3. errorcheck：ADC
  if (adc_raw == 0U || adc_raw > TEMP_ADC_MAX_VALID) {
    if (s_temp_sensor_ok) {
      // fault，error
      s_temp_sensor_ok = false;
      // ：ERROR_REPORT(ERROR_SENSOR_TEMP, "Temp sensor out of range");
    }
    // filter，update
    return NAN;
  }
  // 4. temperature
  float temp;
  temp = G431_NTC_ConvertToTemp(adc_raw);
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
#define G431_ADC_OFFSET_SAMPLE_COUNT 1024U
#define G431_ADC_OFFSET_TIMEOUT_US 1000U

typedef struct {
  uint32_t timeout_cycles;
  uint32_t acquired;
} G431AdcCalibrationContext;

static bool G431_ADC_ReadNextOffsetSample(void *opaque,
                                          MotorAdcCurrentSample *sample) {
  if (opaque == NULL || sample == NULL) {
    return false;
  }

  G431AdcCalibrationContext *context = opaque;
  __HAL_ADC_CLEAR_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOS);
  uint32_t started = DWT->CYCCNT;
  while (!__HAL_ADC_GET_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOS)) {
    if ((uint32_t)(DWT->CYCCNT - started) >= context->timeout_cycles) {
      return false;
    }
  }

  sample->phase_a = (uint16_t)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IA;
  sample->phase_b = (uint16_t)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IB;
  sample->phase_c = (uint16_t)HW_ADC_CURRENT.Instance->HW_ADC_JDR_IC;
  if ((++context->acquired & 0x3FU) == 0U) {
    HAL_WatchdogFeed();
  }
  return true;
}

static bool G431_ADC_CalibrateOffsets(void) {
  const uint32_t injected_it_mask = ADC_IT_JEOC | ADC_IT_JEOS;
  const uint32_t saved_injected_it =
      READ_BIT(HW_ADC_CURRENT.Instance->IER, injected_it_mask);
  __HAL_ADC_DISABLE_IT(&HW_ADC_CURRENT, injected_it_mask);
  __HAL_ADC_CLEAR_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  __DSB();

  G431AdcCalibrationContext context = {
      .timeout_cycles =
          SystemCoreClock / (1000000U / G431_ADC_OFFSET_TIMEOUT_US),
      .acquired = 0U,
  };
  MotorAdcCurrentOffsets offsets = {0};
  bool ok = MotorAdc_CalibrateOffsets(G431_ADC_ReadNextOffsetSample, &context,
                                      G431_ADC_OFFSET_SAMPLE_COUNT, 256U, 3840U,
                                      &offsets);

  __HAL_ADC_CLEAR_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  if (saved_injected_it != 0U) {
    __HAL_ADC_ENABLE_IT(&HW_ADC_CURRENT, saved_injected_it);
  }
  if (!ok) {
    return false;
  }

  current_data.Ia_offset = offsets.phase_a;
  current_data.Ib_offset = offsets.phase_b;
  current_data.Ic_offset = offsets.phase_c;
  return true;
}
static const Motor_HAL_AdcInterface_t g431_adc = {
    .update = G431_ADC_Update, .calibrate_offsets = G431_ADC_CalibrateOffsets};
/* ============================================================================
 * Main Handle Construction
 * ============================================================================
 */
Motor_HAL_Handle_t g431_hal_handle = {
    .pwm = &g431_pwm,
    .adc = &g431_adc,
    .encoder = &g_position_sensor_motor_hal_interface};

#endif /* BOARD_XSTAR */
