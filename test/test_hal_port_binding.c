// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "hal_adc.h"
#include "hal_pwm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

static Motor_HAL_SensorData_t s_next_sample;
static unsigned s_adc_update_count;
static unsigned s_adc_calibrate_count;
static bool s_adc_calibrate_result;

static unsigned s_pwm_start_sampling_count;
static unsigned s_pwm_set_duty_count;
static unsigned s_pwm_enable_count;
static unsigned s_pwm_disable_count;
static unsigned s_pwm_brake_count;
static bool s_pwm_start_sampling_result;
static bool s_pwm_enable_result;
static float s_last_duty_a;
static float s_last_duty_b;
static float s_last_duty_c;

static void fake_adc_update(Motor_HAL_SensorData_t *data) {
  s_adc_update_count++;
  *data = s_next_sample;
}

static bool fake_adc_calibrate_offsets(void) {
  s_adc_calibrate_count++;
  return s_adc_calibrate_result;
}

static bool fake_pwm_start_sampling(void) {
  s_pwm_start_sampling_count++;
  return s_pwm_start_sampling_result;
}

static void fake_pwm_set_duty(float duty_a, float duty_b, float duty_c) {
  s_pwm_set_duty_count++;
  s_last_duty_a = duty_a;
  s_last_duty_b = duty_b;
  s_last_duty_c = duty_c;
}

static bool fake_pwm_enable(void) {
  s_pwm_enable_count++;
  return s_pwm_enable_result;
}

static void fake_pwm_disable(void) { s_pwm_disable_count++; }

static void fake_pwm_brake(void) { s_pwm_brake_count++; }

static const Motor_HAL_AdcInterface_t s_fake_adc = {
    .update = fake_adc_update,
    .calibrate_offsets = fake_adc_calibrate_offsets,
};

static const Motor_HAL_PwmInterface_t s_fake_pwm = {
    .set_duty = fake_pwm_set_duty,
    .start_sampling = fake_pwm_start_sampling,
    .enable = fake_pwm_enable,
    .disable = fake_pwm_disable,
    .brake = fake_pwm_brake,
};

static void reset_fixture(void) {
  assert(MHAL_ADC_Bind(NULL) == -1);
  assert(MHAL_PWM_Bind(NULL) == -1);

  s_next_sample.i_a = 0.0f;
  s_next_sample.i_b = 0.0f;
  s_next_sample.i_c = 0.0f;
  s_next_sample.v_bus = 0.0f;
  s_next_sample.temp = 0.0f;
  s_adc_update_count = 0u;
  s_adc_calibrate_count = 0u;
  s_adc_calibrate_result = true;

  s_pwm_start_sampling_count = 0u;
  s_pwm_set_duty_count = 0u;
  s_pwm_enable_count = 0u;
  s_pwm_disable_count = 0u;
  s_pwm_brake_count = 0u;
  s_pwm_start_sampling_result = true;
  s_pwm_enable_result = true;
  s_last_duty_a = 0.0f;
  s_last_duty_b = 0.0f;
  s_last_duty_c = 0.0f;
}

static void test_unbound_adc_operations_fail(void) {
  float ia = 99.0f;
  float ib = 99.0f;
  float ic = 99.0f;

  reset_fixture();

  assert(MHAL_ADC_Init() == -1);
  assert(MHAL_ADC_Start() == -1);
  assert(MHAL_ADC_Stop() == -1);
  assert(MHAL_ADC_GetCurrent(&ia, &ib, &ic) == -1);
  assert(MHAL_ADC_CalibrateCurrent() == -1);
  assert(MHAL_ADC_GetVbus() == 0.0f);
  assert(MHAL_ADC_GetTemperature() == 0.0f);
  assert(s_adc_update_count == 0u);
}

static void test_unbound_pwm_operations_fail(void) {
  reset_fixture();

  assert(MHAL_PWM_Init() == -1);
  assert(MHAL_PWM_StartSampling() == -1);
  assert(MHAL_PWM_SetDuty(0.1f, 0.2f, 0.3f) == -1);
  assert(MHAL_PWM_Enable() == -1);
  assert(MHAL_PWM_Disable() == -1);
  assert(MHAL_PWM_Brake() == -1);
  assert(s_pwm_start_sampling_count == 0u);
  assert(s_pwm_set_duty_count == 0u);
  assert(s_pwm_enable_count == 0u);
  assert(s_pwm_disable_count == 0u);
  assert(s_pwm_brake_count == 0u);
}

static void test_adc_getters_return_bound_port_update_data(void) {
  float ia = 0.0f;
  float ib = 0.0f;
  float ic = 0.0f;

  reset_fixture();
  s_next_sample.i_a = 1.25f;
  s_next_sample.i_b = -2.5f;
  s_next_sample.i_c = 3.75f;
  s_next_sample.v_bus = 48.0f;
  s_next_sample.temp = 36.5f;

  assert(MHAL_ADC_Bind(&s_fake_adc) == 0);
  assert(MHAL_ADC_Init() == 0);
  assert(MHAL_ADC_GetCurrent(&ia, &ib, &ic) == 0);
  assert(ia == 1.25f);
  assert(ib == -2.5f);
  assert(ic == 3.75f);
  assert(MHAL_ADC_GetVbus() == 48.0f);
  assert(MHAL_ADC_GetTemperature() == 36.5f);
  assert(s_adc_update_count == 3u);
}

static void
test_adc_calibrate_current_returns_failure_when_port_calibration_fails(void) {
  reset_fixture();
  s_adc_calibrate_result = false;

  assert(MHAL_ADC_Bind(&s_fake_adc) == 0);
  assert(MHAL_ADC_CalibrateCurrent() == -1);
  assert(s_adc_calibrate_count == 1u);
}

static void
test_pwm_start_sampling_returns_failure_when_port_start_sampling_fails(void) {
  reset_fixture();
  s_pwm_start_sampling_result = false;

  assert(MHAL_PWM_Bind(&s_fake_pwm) == 0);
  assert(MHAL_PWM_StartSampling() == -1);
  assert(s_pwm_start_sampling_count == 1u);
}

static void test_pwm_enable_returns_failure_when_port_enable_fails(void) {
  reset_fixture();
  s_pwm_enable_result = false;

  assert(MHAL_PWM_Bind(&s_fake_pwm) == 0);
  assert(MHAL_PWM_Enable() == -1);
  assert(s_pwm_enable_count == 1u);
}

static void test_pwm_set_duty_forwards_phase_duty_to_bound_port(void) {
  reset_fixture();

  assert(MHAL_PWM_Bind(&s_fake_pwm) == 0);
  assert(MHAL_PWM_SetDuty(0.125f, 0.5f, 0.875f) == 0);
  assert(s_pwm_set_duty_count == 1u);
  assert(s_last_duty_a == 0.125f);
  assert(s_last_duty_b == 0.5f);
  assert(s_last_duty_c == 0.875f);
}

static void test_pwm_disable_forwards_to_bound_port(void) {
  reset_fixture();

  assert(MHAL_PWM_Bind(&s_fake_pwm) == 0);
  assert(MHAL_PWM_Disable() == 0);
  assert(s_pwm_disable_count == 1u);
}

static void test_pwm_brake_forwards_to_bound_port(void) {
  reset_fixture();

  assert(MHAL_PWM_Bind(&s_fake_pwm) == 0);
  assert(MHAL_PWM_Brake() == 0);
  assert(s_pwm_brake_count == 1u);
}

int main(void) {
  test_unbound_adc_operations_fail();
  test_unbound_pwm_operations_fail();
  test_adc_getters_return_bound_port_update_data();
  test_adc_calibrate_current_returns_failure_when_port_calibration_fails();
  test_pwm_start_sampling_returns_failure_when_port_start_sampling_fails();
  test_pwm_enable_returns_failure_when_port_enable_fails();
  test_pwm_set_duty_forwards_phase_duty_to_bound_port();
  test_pwm_disable_forwards_to_bound_port();
  test_pwm_brake_forwards_to_bound_port();

  puts("HAL port binding tests: PASS");
  return 0;
}
