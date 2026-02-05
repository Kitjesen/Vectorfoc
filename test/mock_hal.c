#include "motor_hal_api.h"
#include <stdio.h>
#include <string.h>

// Mock Data Storage
static float mock_pwm_a = 0.0f;
static float mock_pwm_b = 0.0f;
static float mock_pwm_c = 0.0f;

static float mock_adc_ia = 0.0f;
static float mock_adc_ib = 0.0f;
static float mock_adc_ic = 0.0f;
static float mock_vbus = 24.0f;

static float mock_theta = 0.0f;
static float mock_velocity = 0.0f;

// --- Mock Control Interface ---
void MockHAL_SetCurrents(float ia, float ib, float ic) {
  mock_adc_ia = ia;
  mock_adc_ib = ib;
  mock_adc_ic = ic;
}

void MockHAL_SetEncoder(float theta, float vel) {
  mock_theta = theta;
  mock_velocity = vel;
}

void MockHAL_GetPWM(float *a, float *b, float *c) {
  *a = mock_pwm_a;
  *b = mock_pwm_b;
  *c = mock_pwm_c;
}

// --- HAL Interface Implementation ---

static void pwm_set_duty(float dtc_a, float dtc_b, float dtc_c) {
  mock_pwm_a = dtc_a;
  mock_pwm_b = dtc_b;
  mock_pwm_c = dtc_c;
  // printf("[MOCK HAL] PWM Set: %.2f %.2f %.2f\n", dtc_a, dtc_b, dtc_c);
}

static void pwm_enable(void) { printf("[MOCK HAL] PWM Enabled\n"); }

static void pwm_disable(void) { printf("[MOCK HAL] PWM Disabled\n"); }

static void pwm_brake(void) { printf("[MOCK HAL] PWM Brake\n"); }

static void adc_update(Motor_HAL_SensorData_t *data) {
  data->i_a = mock_adc_ia;
  data->i_b = mock_adc_ib;
  data->i_c = mock_adc_ic;
  data->v_bus = mock_vbus;
  data->temp = 35.0f; // Dummy temp
}

static void adc_calibrate_offsets(void) {
  printf("[MOCK HAL] ADC Calibrate Offsets\n");
}

static void enc_update(void) {
  // No-op for static mock, simulation handles dynamics
}

static void enc_get_data(Motor_HAL_EncoderData_t *data) {
  data->angle_rad = mock_theta;
  data->velocity_rad = mock_velocity;
  data->elec_angle =
      mock_theta; // Simply pass through for now, pole pairs handled in plant
  data->raw_value = 0;
}

static void enc_set_offset(float offset) {
  printf("[MOCK HAL] Encoder Set Offset: %.3f\n", offset);
}

// --- Interface Structs ---
static const Motor_HAL_PwmInterface_t pwm_impl = {
    .set_duty = pwm_set_duty,
    .enable = pwm_enable,
    .disable = pwm_disable,
    .brake = pwm_brake,
};

static const Motor_HAL_AdcInterface_t adc_impl = {
    .update = adc_update,
    .calibrate_offsets = adc_calibrate_offsets,
};

static const Motor_HAL_EncoderInterface_t enc_impl = {
    .update = enc_update,
    .get_data = enc_get_data,
    .set_offset = enc_set_offset,
};

// --- Global Handle ---
Motor_HAL_Handle_t g_mock_hal = {
    .pwm = &pwm_impl,
    .adc = &adc_impl,
    .encoder = &enc_impl,
};

Motor_HAL_Handle_t *MockHAL_GetHandle(void) { return &g_mock_hal; }
