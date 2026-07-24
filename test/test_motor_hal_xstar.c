#include "board_config.h"
#include "motor_adc.h"
#include "motor_hal_api.h"
#include "position_sensor_motor_hal.h"
#include "stm32g4xx_hal.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern Motor_HAL_Handle_t xstar_hal_handle;

TIM_TypeDef mock_tim1_regs;
TIM_TypeDef mock_tim3_regs;
ADC_TypeDef mock_adc1_regs;
ADC_TypeDef mock_adc2_regs;
DWT_Type mock_dwt;
TIM_HandleTypeDef htim1 = {&mock_tim1_regs};
TIM_HandleTypeDef htim3 = {&mock_tim3_regs};
ADC_HandleTypeDef hadc1 = {&mock_adc1_regs};
ADC_HandleTypeDef hadc2 = {&mock_adc2_regs};
FDCAN_HandleTypeDef hfdcan1;
UART_HandleTypeDef huart2;
uint32_t SystemCoreClock = 170000000u;
uint16_t adc1_dma_value[adc1_samples][adc1_channel];
uint16_t adc2_dma_value[adc2_samples][adc2_channel];
CURRENT_DATA current_data;

const Motor_HAL_EncoderInterface_t g_position_sensor_motor_hal_interface = {0};

static uint32_t mock_tick_ms;
static uint32_t pwm_start_mask;
static uint32_t pwmn_start_mask;
static uint32_t pwm_stop_mask;
static uint32_t pwmn_stop_mask;
static uint32_t pwm_start_calls[5];
static uint32_t pwmn_start_calls[5];
static uint32_t fail_pwm_start_channel;
static uint32_t fail_pwmn_start_channel;
static bool adc1_ready;
static bool adc2_ready;
static uint32_t adc_clear_adc1_mask;
static uint32_t adc_clear_adc2_mask;
static uint32_t watchdog_feed_count;
static uint32_t calibration_reader_calls;

uint32_t HAL_GetSystemTick(void) { return mock_tick_ms; }
void HAL_WatchdogFeed(void) { watchdog_feed_count++; }

void mock_hal_tim_set_compare(TIM_HandleTypeDef *handle, uint32_t channel,
                              uint32_t value) {
  assert(handle == &htim1);
  switch (channel) {
  case TIM_CHANNEL_1:
    handle->Instance->CCR1 = value;
    break;
  case TIM_CHANNEL_2:
    handle->Instance->CCR2 = value;
    break;
  case TIM_CHANNEL_3:
    handle->Instance->CCR3 = value;
    break;
  case TIM_CHANNEL_4:
    handle->Instance->CCR4 = value;
    break;
  default:
    assert(false);
  }
}

void mock_hal_adc_clear_flag(ADC_HandleTypeDef *handle, uint32_t flag) {
  handle->Instance->ISR &= ~flag;
  if (handle == &hadc1) {
    adc_clear_adc1_mask |= flag;
  } else if (handle == &hadc2) {
    adc_clear_adc2_mask |= flag;
  } else {
    assert(false);
  }
}

uint32_t mock_hal_adc_get_flag(ADC_HandleTypeDef *handle, uint32_t flag) {
  mock_dwt.CYCCNT += 1000u;
  if (handle == &hadc1) {
    return adc1_ready ? flag : 0u;
  }
  if (handle == &hadc2) {
    return adc2_ready ? flag : 0u;
  }
  assert(false);
  return 0u;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle,
                                    uint32_t channel) {
  assert(handle == &htim1);
  assert(channel < 5u);
  pwm_start_mask |= (1u << channel);
  pwm_start_calls[channel]++;
  return channel == fail_pwm_start_channel ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *handle,
                                       uint32_t channel) {
  assert(handle == &htim1);
  assert(channel < 5u);
  pwmn_start_mask |= (1u << channel);
  pwmn_start_calls[channel]++;
  return channel == fail_pwmn_start_channel ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle,
                                   uint32_t channel) {
  assert(handle == &htim1);
  pwm_stop_mask |= (1u << channel);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *handle,
                                      uint32_t channel) {
  assert(handle == &htim1);
  pwmn_stop_mask |= (1u << channel);
  return HAL_OK;
}

bool MotorAdc_CalibrateOffsets(MotorAdcSampleReader read_next, void *context,
                               uint32_t sample_count, uint16_t valid_min,
                               uint16_t valid_max,
                               MotorAdcCurrentOffsets *result) {
  assert(sample_count == 1024u);
  assert(valid_min == 256u);
  assert(valid_max == 3840u);
  uint64_t sum_a = 0u;
  uint64_t sum_b = 0u;
  uint64_t sum_c = 0u;
  calibration_reader_calls = 0u;
  for (uint32_t i = 0; i < sample_count; ++i) {
    MotorAdcCurrentSample sample = {0};
    if (!read_next(context, &sample)) {
      return false;
    }
    calibration_reader_calls++;
    sum_a += sample.phase_a;
    sum_b += sample.phase_b;
    sum_c += sample.phase_c;
  }
  result->phase_a = (float)sum_a / (float)sample_count;
  result->phase_b = (float)sum_b / (float)sample_count;
  result->phase_c = (float)sum_c / (float)sample_count;
  return result->phase_a > valid_min && result->phase_a < valid_max &&
         result->phase_b > valid_min && result->phase_b < valid_max &&
         result->phase_c > valid_min && result->phase_c < valid_max;
}

static void reset_mocks(void) {
  memset(&mock_tim1_regs, 0, sizeof(mock_tim1_regs));
  memset(&mock_tim3_regs, 0, sizeof(mock_tim3_regs));
  memset(&mock_adc1_regs, 0, sizeof(mock_adc1_regs));
  memset(&mock_adc2_regs, 0, sizeof(mock_adc2_regs));
  memset(&current_data, 0, sizeof(current_data));
  mock_tim1_regs.ARR = 1000u;
  mock_tick_ms = 100u;
  pwm_start_mask = 0u;
  pwmn_start_mask = 0u;
  pwm_stop_mask = 0u;
  pwmn_stop_mask = 0u;
  memset(pwm_start_calls, 0, sizeof(pwm_start_calls));
  memset(pwmn_start_calls, 0, sizeof(pwmn_start_calls));
  fail_pwm_start_channel = 0u;
  fail_pwmn_start_channel = 0u;
  adc1_ready = true;
  adc2_ready = true;
  adc_clear_adc1_mask = 0u;
  adc_clear_adc2_mask = 0u;
  watchdog_feed_count = 0u;
  calibration_reader_calls = 0u;
  mock_dwt.CYCCNT = 0u;
}

static void
start_sampling_disables_phases_zeros_duties_and_starts_only_trigger(void) {
  reset_mocks();
  mock_tim1_regs.CCR1 = 111u;
  mock_tim1_regs.CCR2 = 222u;
  mock_tim1_regs.CCR3 = 333u;

  assert(xstar_hal_handle.pwm->start_sampling());

  assert(mock_tim1_regs.CCR1 == 0u);
  assert(mock_tim1_regs.CCR2 == 0u);
  assert(mock_tim1_regs.CCR3 == 0u);
  assert(pwm_stop_mask == ((1u << TIM_CHANNEL_1) | (1u << TIM_CHANNEL_2) |
                           (1u << TIM_CHANNEL_3)));
  assert(pwmn_stop_mask == ((1u << TIM_CHANNEL_1) | (1u << TIM_CHANNEL_2) |
                            (1u << TIM_CHANNEL_3)));
  assert(pwm_start_mask == (1u << TIM_CHANNEL_4));
  assert(pwmn_start_mask == 0u);
}

static void enable_starts_only_phase_main_and_complementary_channels(void) {
  reset_mocks();

  assert(xstar_hal_handle.pwm->enable());

  assert(pwm_start_mask == ((1u << TIM_CHANNEL_1) | (1u << TIM_CHANNEL_2) |
                            (1u << TIM_CHANNEL_3)));
  assert(pwmn_start_mask == ((1u << TIM_CHANNEL_1) | (1u << TIM_CHANNEL_2) |
                             (1u << TIM_CHANNEL_3)));
  assert(pwm_start_calls[TIM_CHANNEL_4] == 0u);
}

static void enable_rolls_back_when_a_complementary_channel_fails(void) {
  reset_mocks();
  fail_pwmn_start_channel = TIM_CHANNEL_2;

  assert(!xstar_hal_handle.pwm->enable());

  assert(pwm_start_calls[TIM_CHANNEL_1] == 1u);
  assert(pwmn_start_calls[TIM_CHANNEL_1] == 1u);
  assert(pwm_start_calls[TIM_CHANNEL_2] == 1u);
  assert(pwmn_start_calls[TIM_CHANNEL_2] == 1u);
  assert(pwm_start_calls[TIM_CHANNEL_3] == 0u);
  assert(pwm_stop_mask == ((1u << TIM_CHANNEL_1) | (1u << TIM_CHANNEL_2) |
                           (1u << TIM_CHANNEL_3)));
  assert(pwmn_stop_mask == ((1u << TIM_CHANNEL_1) | (1u << TIM_CHANNEL_2) |
                            (1u << TIM_CHANNEL_3)));
}

static void adc_update_maps_adc1_adc2_injected_registers_to_phases(void) {
  reset_mocks();
  Motor_HAL_SensorData_t data = {0};
  current_data.Ia_offset = 2000.0f;
  current_data.Ib_offset = 2000.0f;
  current_data.Ic_offset = 2000.0f;
  mock_adc1_regs.JDR1 = 2100u;
  mock_adc1_regs.JDR2 = 2300u;
  mock_adc1_regs.JDR3 = 3000u;
  mock_adc1_regs.JDR4 = 1432u;
  mock_adc2_regs.JDR1 = 2200u;

  xstar_hal_handle.adc->update(&data);

  assert(fabsf(data.i_a - (100.0f * HW_FAC_CURRENT)) < 1e-5f);
  assert(fabsf(data.i_b - (200.0f * HW_FAC_CURRENT)) < 1e-5f);
  assert(fabsf(data.i_c - (300.0f * HW_FAC_CURRENT)) < 1e-5f);
  assert(fabsf(data.v_bus - (3000.0f * HW_VOLTAGE_FACTOR)) < 1e-4f);
  assert(isfinite(data.temp));
}

static void
calibration_restores_interrupts_and_publishes_dual_adc_offsets(void) {
  reset_mocks();
  mock_adc1_regs.IER = ADC_IT_JEOC | ADC_IT_JEOS | 0x1000u;
  mock_adc2_regs.IER = ADC_IT_JEOS | 0x2000u;
  mock_adc1_regs.JDR1 = 2001u;
  mock_adc1_regs.JDR2 = 2003u;
  mock_adc2_regs.JDR1 = 2002u;

  assert(xstar_hal_handle.adc->calibrate_offsets());

  assert(calibration_reader_calls == 1024u);
  assert(fabsf(current_data.Ia_offset - 2001.0f) < 1e-6f);
  assert(fabsf(current_data.Ib_offset - 2002.0f) < 1e-6f);
  assert(fabsf(current_data.Ic_offset - 2003.0f) < 1e-6f);
  assert((mock_adc1_regs.IER & (ADC_IT_JEOC | ADC_IT_JEOS)) ==
         (ADC_IT_JEOC | ADC_IT_JEOS));
  assert((mock_adc2_regs.IER & (ADC_IT_JEOC | ADC_IT_JEOS)) == ADC_IT_JEOS);
  assert((adc_clear_adc1_mask & (ADC_FLAG_JEOC | ADC_FLAG_JEOS)) ==
         (ADC_FLAG_JEOC | ADC_FLAG_JEOS));
  assert((adc_clear_adc2_mask & (ADC_FLAG_JEOC | ADC_FLAG_JEOS)) ==
         (ADC_FLAG_JEOC | ADC_FLAG_JEOS));
  assert(watchdog_feed_count == 16u);
}

static void
calibration_times_out_waiting_for_both_adcs_and_restores_interrupts(void) {
  reset_mocks();
  current_data.Ia_offset = 11.0f;
  current_data.Ib_offset = 12.0f;
  current_data.Ic_offset = 13.0f;
  mock_adc1_regs.IER = ADC_IT_JEOC;
  mock_adc2_regs.IER = ADC_IT_JEOS;
  adc1_ready = true;
  adc2_ready = false;

  assert(!xstar_hal_handle.adc->calibrate_offsets());

  assert(calibration_reader_calls == 0u);
  assert(fabsf(current_data.Ia_offset - 11.0f) < 1e-6f);
  assert(fabsf(current_data.Ib_offset - 12.0f) < 1e-6f);
  assert(fabsf(current_data.Ic_offset - 13.0f) < 1e-6f);
  assert((mock_adc1_regs.IER & (ADC_IT_JEOC | ADC_IT_JEOS)) == ADC_IT_JEOC);
  assert((mock_adc2_regs.IER & (ADC_IT_JEOC | ADC_IT_JEOS)) == ADC_IT_JEOS);
}

int main(void) {
  start_sampling_disables_phases_zeros_duties_and_starts_only_trigger();
  enable_starts_only_phase_main_and_complementary_channels();
  enable_rolls_back_when_a_complementary_channel_fails();
  adc_update_maps_adc1_adc2_injected_registers_to_phases();
  calibration_restores_interrupts_and_publishes_dual_adc_offsets();
  calibration_times_out_waiting_for_both_adcs_and_restores_interrupts();
  puts("X-STAR Motor HAL direct tests PASSED");
  return 0;
}
