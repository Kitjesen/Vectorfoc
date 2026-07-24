// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef TEST_MOTOR_HAL_G431_STM32G4XX_HAL_H
#define TEST_MOTOR_HAL_G431_STM32G4XX_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  HAL_OK = 0,
  HAL_ERROR = 1,
} HAL_StatusTypeDef;

typedef struct {
  uint32_t ARR;
  uint32_t CCR[5];
} TIM_TypeDef;

typedef struct {
  TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

typedef struct {
  uint32_t ISR;
  uint32_t IER;
  uint32_t JDR1;
  uint32_t JDR2;
  uint32_t JDR3;
  uint32_t JDR4;
} ADC_TypeDef;

typedef struct {
  ADC_TypeDef *Instance;
} ADC_HandleTypeDef;

typedef struct {
  uint32_t CYCCNT;
} DWT_Type;

extern DWT_Type test_dwt;
#define DWT (&test_dwt)

#define TIM_CHANNEL_1 1u
#define TIM_CHANNEL_2 2u
#define TIM_CHANNEL_3 3u
#define TIM_CHANNEL_4 4u

#define ADC_IT_JEOC (1u << 5)
#define ADC_IT_JEOS (1u << 6)
#define ADC_FLAG_JEOC ADC_IT_JEOC
#define ADC_FLAG_JEOS ADC_IT_JEOS

uint32_t test_tim_get_autoreload(TIM_HandleTypeDef *timer);
void test_tim_set_compare(TIM_HandleTypeDef *timer, uint32_t channel,
                          uint32_t value);
HAL_StatusTypeDef test_hal_tim_pwm_start(TIM_HandleTypeDef *timer,
                                         uint32_t channel);
HAL_StatusTypeDef test_hal_tim_pwm_stop(TIM_HandleTypeDef *timer,
                                        uint32_t channel);
HAL_StatusTypeDef test_hal_tim_pwmn_start(TIM_HandleTypeDef *timer,
                                          uint32_t channel);
HAL_StatusTypeDef test_hal_tim_pwmn_stop(TIM_HandleTypeDef *timer,
                                         uint32_t channel);
bool test_hal_adc_get_flag(ADC_HandleTypeDef *adc, uint32_t flag);
void test_hal_adc_clear_flag(ADC_HandleTypeDef *adc, uint32_t flag);
void test_hal_adc_disable_it(ADC_HandleTypeDef *adc, uint32_t mask);
void test_hal_adc_enable_it(ADC_HandleTypeDef *adc, uint32_t mask);

#define __HAL_TIM_GET_AUTORELOAD(timer) test_tim_get_autoreload((timer))
#define __HAL_TIM_SET_COMPARE(timer, channel, value) \
  test_tim_set_compare((timer), (channel), (value))
#define HAL_TIM_PWM_Start(timer, channel) \
  test_hal_tim_pwm_start((timer), (channel))
#define HAL_TIM_PWM_Stop(timer, channel) \
  test_hal_tim_pwm_stop((timer), (channel))
#define HAL_TIMEx_PWMN_Start(timer, channel) \
  test_hal_tim_pwmn_start((timer), (channel))
#define HAL_TIMEx_PWMN_Stop(timer, channel) \
  test_hal_tim_pwmn_stop((timer), (channel))
#define __HAL_ADC_GET_FLAG(adc, flag) test_hal_adc_get_flag((adc), (flag))
#define __HAL_ADC_CLEAR_FLAG(adc, flag) test_hal_adc_clear_flag((adc), (flag))
#define __HAL_ADC_DISABLE_IT(adc, mask) test_hal_adc_disable_it((adc), (mask))
#define __HAL_ADC_ENABLE_IT(adc, mask) test_hal_adc_enable_it((adc), (mask))
#define READ_BIT(reg, mask) ((reg) & (mask))
#define __DSB() ((void)0)

#endif