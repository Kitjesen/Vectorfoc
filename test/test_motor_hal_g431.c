// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "board_config.h"
#include "motor_adc.h"
#include "motor_hal_api.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum {
  EVENT_PWM_START,
  EVENT_PWM_STOP,
  EVENT_PWMN_START,
  EVENT_PWMN_STOP,
  EVENT_SET_COMPARE,
} TestEventKind;

typedef struct {
  TestEventKind kind;
  uint32_t channel;
  uint32_t value;
} TestEvent;

static TIM_TypeDef s_tim1_regs;
static ADC_TypeDef s_adc1_regs;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
DWT_Type test_dwt;
uint32_t SystemCoreClock = 168000000u;
CURRENT_DATA current_data;
uint16_t adc2_dma_value[1][1];

const Motor_HAL_EncoderInterface_t g_position_sensor_motor_hal_interface = {0};
extern Motor_HAL_Handle_t g431_hal_handle;

static TestEvent s_events[64];
static unsigned s_event_count;
static HAL_StatusTypeDef s_pwm_start_result[5];
static HAL_StatusTypeDef s_pwmn_start_result[5];
static bool s_adc_flag_ready;
static bool s_adc_flag_timeout_mode;
static unsigned s_watchdog_feed_count;
static uint32_t s_system_tick;

uint32_t test_tim_get_autoreload(TIM_HandleTypeDef *timer) {
  assert(timer == &htim1);
  return timer->Instance->ARR;
}

static void record_event(TestEventKind kind, uint32_t channel, uint32_t value) {
  assert(s_event_count < (sizeof s_events / sizeof s_events[0]));
  s_events[s_event_count].kind = kind;
  s_events[s_event_count].channel = channel;
  s_events[s_event_count].value = value;
  s_event_count++;
}

void test_tim_set_compare(TIM_HandleTypeDef *timer, uint32_t channel,
                          uint32_t value) {
  assert(timer == &htim1);
  assert(channel <
         (sizeof timer->Instance->CCR / sizeof timer->Instance->CCR[0]));
  timer->Instance->CCR[channel] = value;
  record_event(EVENT_SET_COMPARE, channel, value);
}

HAL_StatusTypeDef test_hal_tim_pwm_start(TIM_HandleTypeDef *timer,
                                         uint32_t channel) {
  assert(timer == &htim1);
  record_event(EVENT_PWM_START, channel, 0u);
  return s_pwm_start_result[channel];
}

HAL_StatusTypeDef test_hal_tim_pwm_stop(TIM_HandleTypeDef *timer,
                                        uint32_t channel) {
  assert(timer == &htim1);
  record_event(EVENT_PWM_STOP, channel, 0u);
  return HAL_OK;
}

HAL_StatusTypeDef test_hal_tim_pwmn_start(TIM_HandleTypeDef *timer,
                                          uint32_t channel) {
  assert(timer == &htim1);
  record_event(EVENT_PWMN_START, channel, 0u);
  return s_pwmn_start_result[channel];
}

HAL_StatusTypeDef test_hal_tim_pwmn_stop(TIM_HandleTypeDef *timer,
                                         uint32_t channel) {
  assert(timer == &htim1);
  record_event(EVENT_PWMN_STOP, channel, 0u);
  return HAL_OK;
}

bool test_hal_adc_get_flag(ADC_HandleTypeDef *adc, uint32_t flag) {
  assert(adc == &hadc1);
  if ((flag & ADC_FLAG_JEOS) == 0u) {
    return false;
  }
  if (s_adc_flag_timeout_mode) {
    test_dwt.CYCCNT += 200000u;
    return false;
  }
  return s_adc_flag_ready;
}

void test_hal_adc_clear_flag(ADC_HandleTypeDef *adc, uint32_t flag) {
  assert(adc == &hadc1);
  adc->Instance->ISR &= ~flag;
}

void test_hal_adc_disable_it(ADC_HandleTypeDef *adc, uint32_t mask) {
  assert(adc == &hadc1);
  adc->Instance->IER &= ~mask;
}

void test_hal_adc_enable_it(ADC_HandleTypeDef *adc, uint32_t mask) {
  assert(adc == &hadc1);
  adc->Instance->IER |= mask;
}

uint32_t HAL_GetSystemTick(void) { return s_system_tick; }

void HAL_WatchdogFeed(void) { s_watchdog_feed_count++; }

bool MotorAdc_CalibrateOffsets(MotorAdcSampleReader read_next, void *context,
                               uint32_t sample_count, uint16_t valid_min,
                               uint16_t valid_max,
                               MotorAdcCurrentOffsets *result) {
  uint32_t sum_a = 0u;
  uint32_t sum_b = 0u;
  uint32_t sum_c = 0u;
  for (uint32_t i = 0u; i < sample_count; ++i) {
    MotorAdcCurrentSample sample = {0};
    if (!read_next(context, &sample)) {
      return false;
    }
    if (sample.phase_a <= valid_min || sample.phase_a >= valid_max ||
        sample.phase_b <= valid_min || sample.phase_b >= valid_max ||
        sample.phase_c <= valid_min || sample.phase_c >= valid_max) {
      return false;
    }
    sum_a += sample.phase_a;
    sum_b += sample.phase_b;
    sum_c += sample.phase_c;
  }
  result->phase_a = (float)sum_a / (float)sample_count;
  result->phase_b = (float)sum_b / (float)sample_count;
  result->phase_c = (float)sum_c / (float)sample_count;
  return true;
}

static void reset_fixture(void) {
  memset(&s_tim1_regs, 0, sizeof s_tim1_regs);
  memset(&s_adc1_regs, 0, sizeof s_adc1_regs);
  memset(&current_data, 0, sizeof current_data);
  memset(adc2_dma_value, 0, sizeof adc2_dma_value);
  memset(s_events, 0, sizeof s_events);
  s_event_count = 0u;
  for (unsigned i = 0u; i < 5u; ++i) {
    s_pwm_start_result[i] = HAL_OK;
    s_pwmn_start_result[i] = HAL_OK;
  }
  htim1.Instance = &s_tim1_regs;
  hadc1.Instance = &s_adc1_regs;
  s_tim1_regs.ARR = 4200u;
  s_adc_flag_ready = true;
  s_adc_flag_timeout_mode = false;
  s_watchdog_feed_count = 0u;
  s_system_tick = 100u;
  test_dwt.CYCCNT = 0u;
}

static unsigned count_events(TestEventKind kind, uint32_t channel) {
  unsigned count = 0u;
  for (unsigned i = 0u; i < s_event_count; ++i) {
    if (s_events[i].kind == kind && s_events[i].channel == channel) {
      count++;
    }
  }
  return count;
}

static void assert_disable_sequence_at(unsigned start) {
  assert(s_events[start + 0u].kind == EVENT_PWM_STOP);
  assert(s_events[start + 0u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[start + 1u].kind == EVENT_PWM_STOP);
  assert(s_events[start + 1u].channel == HW_PWM_CH_PHASE_B);
  assert(s_events[start + 2u].kind == EVENT_PWM_STOP);
  assert(s_events[start + 2u].channel == HW_PWM_CH_PHASE_C);
  assert(s_events[start + 3u].kind == EVENT_PWMN_STOP);
  assert(s_events[start + 3u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[start + 4u].kind == EVENT_PWMN_STOP);
  assert(s_events[start + 4u].channel == HW_PWM_CH_PHASE_B);
  assert(s_events[start + 5u].kind == EVENT_PWMN_STOP);
  assert(s_events[start + 5u].channel == HW_PWM_CH_PHASE_C);
}

static void
start_sampling_disables_outputs_zeros_duties_and_starts_only_trigger(void) {
  reset_fixture();
  s_tim1_regs.CCR[HW_PWM_CH_PHASE_A] = 100u;
  s_tim1_regs.CCR[HW_PWM_CH_PHASE_B] = 200u;
  s_tim1_regs.CCR[HW_PWM_CH_PHASE_C] = 300u;

  assert(g431_hal_handle.pwm->start_sampling());

  assert(s_event_count == 10u);
  assert_disable_sequence_at(0u);
  assert(s_events[6u].kind == EVENT_SET_COMPARE);
  assert(s_events[6u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[6u].value == 0u);
  assert(s_events[7u].kind == EVENT_SET_COMPARE);
  assert(s_events[7u].channel == HW_PWM_CH_PHASE_B);
  assert(s_events[7u].value == 0u);
  assert(s_events[8u].kind == EVENT_SET_COMPARE);
  assert(s_events[8u].channel == HW_PWM_CH_PHASE_C);
  assert(s_events[8u].value == 0u);
  assert(s_events[9u].kind == EVENT_PWM_START);
  assert(s_events[9u].channel == HW_PWM_CH_TRIG);
  assert(count_events(EVENT_PWM_START, HW_PWM_CH_PHASE_A) == 0u);
  assert(count_events(EVENT_PWMN_START, HW_PWM_CH_PHASE_A) == 0u);
  assert(s_tim1_regs.CCR[HW_PWM_CH_PHASE_A] == 0u);
  assert(s_tim1_regs.CCR[HW_PWM_CH_PHASE_B] == 0u);
  assert(s_tim1_regs.CCR[HW_PWM_CH_PHASE_C] == 0u);
}

static void enable_starts_only_phase_main_and_complementary_channels(void) {
  reset_fixture();

  assert(g431_hal_handle.pwm->enable());

  assert(s_event_count == 6u);
  assert(s_events[0u].kind == EVENT_PWM_START);
  assert(s_events[0u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[1u].kind == EVENT_PWMN_START);
  assert(s_events[1u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[2u].kind == EVENT_PWM_START);
  assert(s_events[2u].channel == HW_PWM_CH_PHASE_B);
  assert(s_events[3u].kind == EVENT_PWMN_START);
  assert(s_events[3u].channel == HW_PWM_CH_PHASE_B);
  assert(s_events[4u].kind == EVENT_PWM_START);
  assert(s_events[4u].channel == HW_PWM_CH_PHASE_C);
  assert(s_events[5u].kind == EVENT_PWMN_START);
  assert(s_events[5u].channel == HW_PWM_CH_PHASE_C);
  assert(count_events(EVENT_PWM_START, HW_PWM_CH_TRIG) == 0u);
}

static void enable_rolls_back_when_phase_main_start_fails(void) {
  reset_fixture();
  s_pwm_start_result[HW_PWM_CH_PHASE_B] = HAL_ERROR;

  assert(!g431_hal_handle.pwm->enable());

  assert(s_event_count == 9u);
  assert(s_events[0u].kind == EVENT_PWM_START);
  assert(s_events[0u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[1u].kind == EVENT_PWMN_START);
  assert(s_events[1u].channel == HW_PWM_CH_PHASE_A);
  assert(s_events[2u].kind == EVENT_PWM_START);
  assert(s_events[2u].channel == HW_PWM_CH_PHASE_B);
  assert_disable_sequence_at(3u);
}

static void enable_rolls_back_when_phase_complementary_start_fails(void) {
  reset_fixture();
  s_pwmn_start_result[HW_PWM_CH_PHASE_C] = HAL_ERROR;

  assert(!g431_hal_handle.pwm->enable());

  assert(s_event_count == 12u);
  assert(s_events[4u].kind == EVENT_PWM_START);
  assert(s_events[4u].channel == HW_PWM_CH_PHASE_C);
  assert(s_events[5u].kind == EVENT_PWMN_START);
  assert(s_events[5u].channel == HW_PWM_CH_PHASE_C);
  assert_disable_sequence_at(6u);
}

static void adc_update_uses_phase_jdr_mapping_and_offsets(void) {
  reset_fixture();
  current_data.Ia_offset = 1000.0f;
  current_data.Ib_offset = 1100.0f;
  current_data.Ic_offset = 1200.0f;
  s_adc1_regs.JDR1 = 1250u;
  s_adc1_regs.JDR2 = 1175u;
  s_adc1_regs.JDR3 = 1040u;
  s_adc1_regs.JDR4 = 2048u;
  adc2_dma_value[0][adc2_ch12] = 2048u;

  Motor_HAL_SensorData_t data = {0};
  g431_hal_handle.adc->update(&data);

  assert(fabsf(data.i_a - (40.0f * HW_FAC_CURRENT)) < 0.000001f);
  assert(fabsf(data.i_b - (75.0f * HW_FAC_CURRENT)) < 0.000001f);
  assert(fabsf(data.i_c - (50.0f * HW_FAC_CURRENT)) < 0.000001f);
  assert(fabsf(data.v_bus - (2048.0f * HW_VOLTAGE_FACTOR)) < 0.0001f);
  assert(!isnan(data.temp));
}

static void calibration_timeout_restores_saved_injected_interrupts(void) {
  reset_fixture();
  s_adc1_regs.IER = ADC_IT_JEOC | ADC_IT_JEOS;
  s_adc_flag_ready = false;
  s_adc_flag_timeout_mode = true;
  current_data.Ia_offset = 11.0f;
  current_data.Ib_offset = 22.0f;
  current_data.Ic_offset = 33.0f;

  assert(!g431_hal_handle.adc->calibrate_offsets());

  assert((s_adc1_regs.IER & (ADC_IT_JEOC | ADC_IT_JEOS)) ==
         (ADC_IT_JEOC | ADC_IT_JEOS));
  assert(current_data.Ia_offset == 11.0f);
  assert(current_data.Ib_offset == 22.0f);
  assert(current_data.Ic_offset == 33.0f);
}

static void calibration_publishes_offsets_after_successful_samples(void) {
  reset_fixture();
  s_adc1_regs.IER = ADC_IT_JEOS;
  s_adc1_regs.JDR1 = 1300u;
  s_adc1_regs.JDR2 = 1200u;
  s_adc1_regs.JDR3 = 1100u;

  assert(g431_hal_handle.adc->calibrate_offsets());

  assert(current_data.Ia_offset == 1100.0f);
  assert(current_data.Ib_offset == 1200.0f);
  assert(current_data.Ic_offset == 1300.0f);
  assert((s_adc1_regs.IER & ADC_IT_JEOS) == ADC_IT_JEOS);
  assert(s_watchdog_feed_count == 16u);
}

int main(void) {
  start_sampling_disables_outputs_zeros_duties_and_starts_only_trigger();
  enable_starts_only_phase_main_and_complementary_channels();
  enable_rolls_back_when_phase_main_start_fails();
  enable_rolls_back_when_phase_complementary_start_fails();
  adc_update_uses_phase_jdr_mapping_and_offsets();
  calibration_timeout_restores_saved_injected_interrupts();
  calibration_publishes_offsets_after_successful_samples();

  puts("G431 motor HAL direct tests: PASS");
  return 0;
}