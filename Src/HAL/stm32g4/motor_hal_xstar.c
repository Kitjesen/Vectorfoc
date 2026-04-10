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
#include "board_config_xstar.h"
#include "motor_hal_api.h"
#include "hall_encoder.h"
#include "abz_encoder.h"
#include "motor_adc.h"       /* current_data */
#include "hal_abstraction.h" /* HAL_GetSystemTick() */
#include <math.h>

/* ==========================================================================
   电流/电压转换常量（基于 board_config_xstar.h）
   ========================================================================== */

/* A/LSB：偏置去零后，每 LSB 对应的电流值 */
/* I = (ADC_raw - offset) × FAC_CURRENT */
#define XSTAR_FAC_CURRENT   HW_FAC_CURRENT

/* V/LSB：母线电压转换系数 */
#define XSTAR_VOLTAGE_FACTOR HW_VOLTAGE_FACTOR

/* ==========================================================================
   温度转换（Steinhart-Hart B参数方程）
   电路：+3.3V → NTC → TEMP(ADC) → R69(10kΩ) → GND
   ADC_raw = 4095 × R_down / (R_ntc + R_down)
   → R_ntc = R_down × (4095 - raw) / raw
   → T[K] = B / (ln(R_ntc / R25) + B / T25)
   ========================================================================== */
#define XSTAR_TEMP_T25_K    298.15f     /* 25°C in Kelvin */
#define XSTAR_TEMP_UPDATE_MS 20         /* 温度更新周期 50Hz */
#define XSTAR_TEMP_LPF_ALPHA 0.1f       /* 低通滤波系数 */
#define XSTAR_TEMP_ADC_MIN   50         /* ADC有效下限（开路保护）*/
#define XSTAR_TEMP_ADC_MAX   4050       /* ADC有效上限（短路保护）*/

static uint32_t s_last_temp_ms   = 0;
static float    s_temp_filtered  = 25.0f;

static float XStar_NTC_ConvertToTemp(uint16_t adc_raw) {
    /* 范围检查 */
    if (adc_raw < XSTAR_TEMP_ADC_MIN || adc_raw > XSTAR_TEMP_ADC_MAX) {
        return s_temp_filtered;
    }
    /* R_ntc = R_down × (ADC_MAX - raw) / raw */
    float r_ntc = HW_NTC_PULLDOWN * (float)(HW_ADC_RESOLUTION - adc_raw) / (float)adc_raw;
    /* Steinhart-Hart简化：T = B / (ln(R/R25) + B/T25) */
    float t_kelvin = HW_NTC_B_VALUE / (logf(r_ntc / HW_NTC_R25) + HW_NTC_B_VALUE / XSTAR_TEMP_T25_K);
    return t_kelvin - 273.15f;
}

static float XStar_ReadTemperature(uint16_t adc_raw) {
    uint32_t now = HAL_GetSystemTick();
    if (now - s_last_temp_ms < XSTAR_TEMP_UPDATE_MS) {
        return s_temp_filtered;
    }
    s_last_temp_ms = now;
    float temp = XStar_NTC_ConvertToTemp(adc_raw);
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
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_U, (uint16_t)(dtc_a * arr));
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_V, (uint16_t)(dtc_b * arr));
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_W, (uint16_t)(dtc_c * arr));
}

static void XStar_PWM_Enable(void) {
    /* Keep CH4 (ADC trigger) always active so TIM1 never stops.
     * HAL_TIM_PWM_Stop for CH1/2/3 only stops the timer when ALL CCxE=0.
     * With CC4E=1, __HAL_TIM_DISABLE's precondition fails → TIM1 keeps counting
     * → ADC ISR keeps firing → StateMachine_Update always runs. */
    HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_TRIG);
    HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_U);
    HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_V);
    HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_W);
    HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, HW_PWM_CH_U);
    HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, HW_PWM_CH_V);
    HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, HW_PWM_CH_W);
}

static void XStar_PWM_Disable(void) {
    HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_U);
    HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_V);
    HAL_TIM_PWM_Stop(&HW_PWM_TIMER, HW_PWM_CH_W);
    HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_U);
    HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_V);
    HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, HW_PWM_CH_W);
}

static void XStar_PWM_Brake(void) {
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_U, 0);
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_V, 0);
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_W, 0);
    XStar_PWM_Enable();
}

static const Motor_HAL_PwmInterface_t xstar_pwm = {
    .set_duty = XStar_PWM_SetDuty,
    .enable   = XStar_PWM_Enable,
    .disable  = XStar_PWM_Disable,
    .brake    = XStar_PWM_Brake,
};

/* ==========================================================================
   ADC 接口实现
   ADC1 注入组：JDR1=Iu, JDR2=Iw, JDR3=Vbus, JDR4=Temp
   ADC2 注入组：JDR1=Iv
   ========================================================================== */

static void XStar_ADC_Update(Motor_HAL_SensorData_t *data) {
    /* 直接读取注入寄存器（在TIM1_CC4触发后、ISR中调用时数据已就绪） */
    float adc_iu   = (float)hadc1.Instance->HW_ADC1_JDR_IU;
    float adc_iw   = (float)hadc1.Instance->HW_ADC1_JDR_IW;
    float adc_vbus = (float)hadc1.Instance->HW_ADC1_JDR_VBUS;
    float adc_temp = (float)hadc1.Instance->HW_ADC1_JDR_TEMP;
    float adc_iv   = (float)hadc2.Instance->HW_ADC2_JDR_IV;

    /* 电流（差分放大器：偏置1.65V已通过offset校准消除） */
    data->i_a = (adc_iu - current_data.Ia_offset) * XSTAR_FAC_CURRENT;
    data->i_b = (adc_iv - current_data.Ib_offset) * XSTAR_FAC_CURRENT;
    data->i_c = (adc_iw - current_data.Ic_offset) * XSTAR_FAC_CURRENT;

    /* 母线电压 */
    data->v_bus = adc_vbus * XSTAR_VOLTAGE_FACTOR;

    /* 温度（限速更新+低通滤波） */
    data->temp = XStar_ReadTemperature((uint16_t)adc_temp);
}

static void XStar_ADC_CalibrateOffsets(void) {
    /* 静止时采集1000次，取均值作为零偏
     * 偏置电压1.65V对应ADC值约2047，实际因OPAMP和电路略有偏差 */
    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    const int N = 1000;
    for (int i = 0; i < N; i++) {
        HAL_Delay(1);
        sum_u += (uint32_t)hadc1.Instance->HW_ADC1_JDR_IU;
        sum_v += (uint32_t)hadc2.Instance->HW_ADC2_JDR_IV;
        sum_w += (uint32_t)hadc1.Instance->HW_ADC1_JDR_IW;
    }
    current_data.Ia_offset = (float)sum_u / N;
    current_data.Ib_offset = (float)sum_v / N;
    current_data.Ic_offset = (float)sum_w / N;
}

static const Motor_HAL_AdcInterface_t xstar_adc = {
    .update            = XStar_ADC_Update,
    .calibrate_offsets = XStar_ADC_CalibrateOffsets,
};

/* ==========================================================================
   编码器接口（转接到 hall_encoder.c）
   ========================================================================== */

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
extern const Motor_HAL_EncoderInterface_t g_hall_encoder_interface;
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
extern const Motor_HAL_EncoderInterface_t g_abz_encoder_interface;
#else
#error "Unsupported X-STAR position sensor mode"
#endif

/* ==========================================================================
   主 HAL Handle（注册给 motor_data）
   ========================================================================== */
Motor_HAL_Handle_t xstar_hal_handle = {
    .pwm     = &xstar_pwm,
    .adc     = &xstar_adc,
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    .encoder = &g_hall_encoder_interface,
#else
    .encoder = &g_abz_encoder_interface,
#endif
};
