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
 * @file motor_hal_xstar.c
 * @brief X-STAR-S STM32G431RBT6 开发板 Motor HAL 实现
 *
 * 对接 Motor_HAL_Handle_t 接口，适配 X-STAR-S 硬件：
 *   - PWM：TIM1，V下桥 PA12（与VectorFOC的PB14不同）
 *   - 电流：ADC1注入（Iu, Iw, Vbus, Temp） + ADC2注入（Iv），差分OPAMP 10倍增益
 *   - 温度：NCP18WB473J03RB（47kΩ@25°C, B=3950），Steinhart-Hart公式
 *   - 编码器：霍尔传感器（hall_encoder.c）
 */
#ifdef BOARD_XSTAR

#include "board_config.h"
#include "bsp_dwt.h"
#include "config.h"
#include "hal_abstraction.h" /* HAL_GetSystemTick() */
#include "motor_adc.h"       /* current_data */
#include "motor_hal_api.h"
#include "position_sensor_motor_hal.h"
#include <math.h>

/* ==========================================================================
   电流/电压转换常量（基于统一 board_config.h 板级契约）
   ========================================================================== */

/* A/LSB：偏置去零后，每 LSB 对应的电流值 */
/* I = (ADC_raw - offset) × FAC_CURRENT */
#define XSTAR_FAC_CURRENT HW_FAC_CURRENT

/* V/LSB：母线电压转换系数 */
#define XSTAR_VOLTAGE_FACTOR HW_VOLTAGE_FACTOR

/* ==========================================================================
   温度转换（Steinhart-Hart B参数方程）
   电路：+3.3V → NTC → TEMP(ADC) → R69(10kΩ) → GND
   ADC_raw = 4095 × R_down / (R_ntc + R_down)
   → R_ntc = R_down × (4095 - raw) / raw
   → T[K] = B / (ln(R_ntc / R25) + B / T25)
   ========================================================================== */
#define XSTAR_TEMP_T25_K 298.15f  /* 25°C in Kelvin */
#define XSTAR_TEMP_UPDATE_MS 20   /* 温度更新周期 50Hz */
#define XSTAR_TEMP_LPF_ALPHA 0.1f /* 低通滤波系数 */
#define XSTAR_TEMP_ADC_MIN 50     /* ADC有效下限（开路保护）*/
#define XSTAR_TEMP_ADC_MAX 4050   /* ADC有效上限（短路保护）*/

static uint32_t s_last_temp_ms = 0;
static float s_temp_filtered = 25.0f;

static float XStar_NTC_ConvertToTemp(uint16_t adc_raw) {
  /* 范围检查 */
  if (adc_raw < XSTAR_TEMP_ADC_MIN || adc_raw > XSTAR_TEMP_ADC_MAX) {
    return NAN;
  }
  /* R_ntc = R_down × (ADC_MAX - raw) / raw */
  float r_ntc =
      HW_NTC_PULLDOWN * (float)(HW_ADC_RESOLUTION - adc_raw) / (float)adc_raw;
  /* Steinhart-Hart简化：T = B / (ln(R/R25) + B/T25) */
  float t_kelvin = HW_NTC_B_VALUE / (logf(r_ntc / HW_NTC_R25) +
                                     HW_NTC_B_VALUE / XSTAR_TEMP_T25_K);
  return t_kelvin - 273.15f;
}

static float XStar_ReadTemperature(uint16_t adc_raw) {
  uint32_t now = HAL_GetSystemTick();
  if (now - s_last_temp_ms < XSTAR_TEMP_UPDATE_MS) {
    return s_temp_filtered;
  }
  s_last_temp_ms = now;
  float temp = XStar_NTC_ConvertToTemp(adc_raw);
  if (!isfinite(temp)) {
    /* A disconnected/shorted NTC must fail safe in fault detection. */
    return NAN;
  }
  s_temp_filtered += XSTAR_TEMP_LPF_ALPHA * (temp - s_temp_filtered);
  return s_temp_filtered;
}

/* ==========================================================================
   PWM 接口实现
   主要差异：V下桥为 PA12（TIM1_CH2N），原版为 PB14
   ========================================================================== */

static void XStar_PWM_SetDuty(float dtc_a, float dtc_b, float dtc_c) {
  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&HW_PWM_TIMER);
  /* 相序：set_duty(Ta=U, Tb=V, Tc=W) → CH_U, CH_V, CH_W */
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A,
                        (uint16_t)(dtc_a * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B,
                        (uint16_t)(dtc_b * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C,
                        (uint16_t)(dtc_c * arr));
}

static void XStar_PWM_Disable(void);
static bool XStar_PWM_StartSampling(void) {
  XStar_PWM_Disable();
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A, 0U);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B, 0U);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C, 0U);
  return HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_TRIG) == HAL_OK;
}
static bool XStar_PWM_StartChannel(uint32_t channel) {
  return HAL_TIM_PWM_Start(&HW_PWM_TIMER, channel) == HAL_OK &&
         HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, channel) == HAL_OK;
}

static bool XStar_PWM_Enable(void) {
  /* Keep CH4 (ADC trigger) always active so TIM1 never stops.
   * HAL_TIM_PWM_Stop for CH1/2/3 only stops the timer when ALL CCxE=0.
   * With CC4E=1, __HAL_TIM_DISABLE's precondition fails → TIM1 keeps counting
   * → ADC ISR keeps firing → StateMachine_Update always runs. */
  if (!XStar_PWM_StartChannel(HW_PWM_CH_PHASE_A) ||
      !XStar_PWM_StartChannel(HW_PWM_CH_PHASE_B) ||
      !XStar_PWM_StartChannel(HW_PWM_CH_PHASE_C)) {
    XStar_PWM_Disable();
    return false;
  }
  return true;
}

static void XStar_PWM_Disable(void) {
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A);
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B);
  HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B);
  HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C);
}

static void XStar_PWM_Brake(void) {
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_A, 0);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_B, 0);
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_PHASE_C, 0);
  XStar_PWM_Disable();
}

static const Motor_HAL_PwmInterface_t xstar_pwm = {
    .set_duty = XStar_PWM_SetDuty,
    .start_sampling = XStar_PWM_StartSampling,
    .enable = XStar_PWM_Enable,
    .disable = XStar_PWM_Disable,
    .brake = XStar_PWM_Brake,
};

/* ==========================================================================
   ADC 接口实现
   ADC1 注入组：JDR1=Iu, JDR2=Iw, JDR3=Vbus, JDR4=Temp
   ADC2 注入组：JDR1=Iv
   ========================================================================== */

static void XStar_ADC_Update(Motor_HAL_SensorData_t *data) {
  /* 直接读取注入寄存器（在TIM1_CC4触发后、ISR中调用时数据已就绪） */
  float adc_iu = (float)hadc1.Instance->HW_ADC1_JDR_IU;
  float adc_iw = (float)hadc1.Instance->HW_ADC1_JDR_IW;
  float adc_vbus = (float)hadc1.Instance->HW_ADC1_JDR_VBUS;
  float adc_temp = (float)hadc1.Instance->HW_ADC1_JDR_TEMP;
  float adc_iv = (float)hadc2.Instance->HW_ADC2_JDR_IV;

  /* 电流（差分放大器：偏置1.65V已通过offset校准消除） */
  data->i_a = (adc_iu - current_data.Ia_offset) * XSTAR_FAC_CURRENT;
  data->i_b = (adc_iv - current_data.Ib_offset) * XSTAR_FAC_CURRENT;
  data->i_c = (adc_iw - current_data.Ic_offset) * XSTAR_FAC_CURRENT;

  /* 母线电压 */
  data->v_bus = adc_vbus * XSTAR_VOLTAGE_FACTOR;

  /* 温度（限速更新+低通滤波） */
  data->temp = XStar_ReadTemperature((uint16_t)adc_temp);
}

#define XSTAR_ADC_OFFSET_SAMPLE_COUNT 1024U
#define XSTAR_ADC_OFFSET_TIMEOUT_US 1000U

typedef struct {
  uint32_t timeout_cycles;
  uint32_t acquired;
} XStarAdcCalibrationContext;

static bool XStar_ADC_ReadNextOffsetSample(void *opaque,
                                           MotorAdcCurrentSample *sample) {
  if (opaque == NULL || sample == NULL) {
    return false;
  }

  XStarAdcCalibrationContext *context = opaque;
  __HAL_ADC_CLEAR_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOS);
  __HAL_ADC_CLEAR_FLAG(&HW_ADC2_CURRENT, ADC_FLAG_JEOS);
  uint32_t started = DWT->CYCCNT;
  while (!__HAL_ADC_GET_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOS) ||
         !__HAL_ADC_GET_FLAG(&HW_ADC2_CURRENT, ADC_FLAG_JEOS)) {
    if ((uint32_t)(DWT->CYCCNT - started) >= context->timeout_cycles) {
      return false;
    }
  }

  sample->phase_a = (uint16_t)hadc1.Instance->HW_ADC1_JDR_IU;
  sample->phase_b = (uint16_t)hadc2.Instance->HW_ADC2_JDR_IV;
  sample->phase_c = (uint16_t)hadc1.Instance->HW_ADC1_JDR_IW;
  if ((++context->acquired & 0x3FU) == 0U) {
    HAL_WatchdogFeed();
  }
  return true;
}

static bool XStar_ADC_CalibrateOffsets(void) {
  const uint32_t injected_it_mask = ADC_IT_JEOC | ADC_IT_JEOS;
  const uint32_t saved_adc1_it =
      READ_BIT(HW_ADC_CURRENT.Instance->IER, injected_it_mask);
  const uint32_t saved_adc2_it =
      READ_BIT(HW_ADC2_CURRENT.Instance->IER, injected_it_mask);
  __HAL_ADC_DISABLE_IT(&HW_ADC_CURRENT, injected_it_mask);
  __HAL_ADC_DISABLE_IT(&HW_ADC2_CURRENT, injected_it_mask);
  __HAL_ADC_CLEAR_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  __HAL_ADC_CLEAR_FLAG(&HW_ADC2_CURRENT, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  __DSB();

  XStarAdcCalibrationContext context = {
      .timeout_cycles =
          SystemCoreClock / (1000000U / XSTAR_ADC_OFFSET_TIMEOUT_US),
      .acquired = 0U,
  };
  MotorAdcCurrentOffsets offsets = {0};
  bool ok = MotorAdc_CalibrateOffsets(XStar_ADC_ReadNextOffsetSample, &context,
                                      XSTAR_ADC_OFFSET_SAMPLE_COUNT, 256U,
                                      3840U, &offsets);

  __HAL_ADC_CLEAR_FLAG(&HW_ADC_CURRENT, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  __HAL_ADC_CLEAR_FLAG(&HW_ADC2_CURRENT, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  if (saved_adc1_it != 0U) {
    __HAL_ADC_ENABLE_IT(&HW_ADC_CURRENT, saved_adc1_it);
  }
  if (saved_adc2_it != 0U) {
    __HAL_ADC_ENABLE_IT(&HW_ADC2_CURRENT, saved_adc2_it);
  }
  if (!ok) {
    return false;
  }

  current_data.Ia_offset = offsets.phase_a;
  current_data.Ib_offset = offsets.phase_b;
  current_data.Ic_offset = offsets.phase_c;
  return true;
}
static const Motor_HAL_AdcInterface_t xstar_adc = {
    .update = XStar_ADC_Update,
    .calibrate_offsets = XStar_ADC_CalibrateOffsets,
};

/* ==========================================================================
   主 HAL Handle（注册给 motor_data）
   ========================================================================== */
Motor_HAL_Handle_t xstar_hal_handle = {
    .pwm = &xstar_pwm,
    .adc = &xstar_adc,
    .encoder = &g_position_sensor_motor_hal_interface,
};

#endif /* BOARD_XSTAR */
