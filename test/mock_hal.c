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

#include "motor_hal_api.h"
#include "fsm.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Mock Data Storage
static float mock_pwm_a = 0.0f;
static float mock_pwm_b = 0.0f;
static float mock_pwm_c = 0.0f;
static int mock_pwm_set_duty_count = 0;
static int mock_pwm_enable_count = 0;
static int mock_pwm_disable_count = 0;
static int mock_pwm_brake_count = 0;

static float mock_adc_ia = 0.0f;
static float mock_adc_ib = 0.0f;
static float mock_adc_ic = 0.0f;
static float mock_vbus = 24.0f;
static float mock_noise_sigma = 0.0f;
static uint32_t mock_noise_state = 0x12345678u;

static float mock_position = 0.0f;
static float mock_angle = 0.0f;
static float mock_velocity = 0.0f;
static float mock_electrical_angle = 0.0f;
static bool mock_encoder_valid = true;

// --- Mock Control Interface ---
void MockHAL_Reset(void) {
  mock_pwm_a = 0.0f;
  mock_pwm_b = 0.0f;
  mock_pwm_c = 0.0f;
  mock_pwm_set_duty_count = 0;
  mock_pwm_enable_count = 0;
  mock_pwm_disable_count = 0;
  mock_pwm_brake_count = 0;
  mock_adc_ia = 0.0f;
  mock_adc_ib = 0.0f;
  mock_adc_ic = 0.0f;
  mock_vbus = 24.0f;
  mock_noise_sigma = 0.0f;
  mock_noise_state = 0x12345678u;
  mock_position = 0.0f;
  mock_angle = 0.0f;
  mock_velocity = 0.0f;
  mock_electrical_angle = 0.0f;
  mock_encoder_valid = true;
}

int MockHAL_GetPwmSetDutyCount(void) { return mock_pwm_set_duty_count; }
int MockHAL_GetPwmEnableCount(void) { return mock_pwm_enable_count; }
int MockHAL_GetPwmDisableCount(void) { return mock_pwm_disable_count; }
int MockHAL_GetPwmBrakeCount(void) { return mock_pwm_brake_count; }

void MockHAL_SetNoiseLevel(float sigma) {
  mock_noise_sigma = sigma;
}

void MockHAL_SetCurrents(float ia, float ib, float ic) {
  mock_adc_ia = ia;
  mock_adc_ib = ib;
  mock_adc_ic = ic;
}

void MockHAL_SetEncoder(float position, float angle, float velocity,
                        float electrical_angle) {
  mock_position = position;
  mock_angle = angle;
  mock_velocity = velocity;
  mock_electrical_angle = electrical_angle;
  mock_encoder_valid = true;
}

void MockHAL_SetEncoderValid(bool valid) { mock_encoder_valid = valid; }

void MockHAL_GetPWM(float *a, float *b, float *c) {
  *a = mock_pwm_a;
  *b = mock_pwm_b;
  *c = mock_pwm_c;
}

// --- HAL Interface Implementation ---

static float mock_noise_sample(void) {
  mock_noise_state = mock_noise_state * 1664525u + 1013904223u;
  uint32_t centered = (mock_noise_state >> 8) & 0xFFFFu;
  return ((float)centered / 32767.5f) - 1.0f;
}

static void pwm_set_duty(float dtc_a, float dtc_b, float dtc_c) {
  mock_pwm_a = dtc_a;
  mock_pwm_b = dtc_b;
  mock_pwm_c = dtc_c;
  mock_pwm_set_duty_count++;
  // printf("[MOCK HAL] PWM Set: %.2f %.2f %.2f\n", dtc_a, dtc_b, dtc_c);
}

static bool pwm_enable(void) {
  mock_pwm_enable_count++;
  printf("[MOCK HAL] PWM Enabled\n");
  return true;
}

static void pwm_disable(void) {
  mock_pwm_disable_count++;
  printf("[MOCK HAL] PWM Disabled\n");
}

static void pwm_brake(void) {
  mock_pwm_brake_count++;
  printf("[MOCK HAL] PWM Brake\n");
}

static void adc_update(Motor_HAL_SensorData_t *data) {
  data->i_a = mock_adc_ia + mock_noise_sigma * mock_noise_sample();
  data->i_b = mock_adc_ib + mock_noise_sigma * mock_noise_sample();
  data->i_c = mock_adc_ic + mock_noise_sigma * mock_noise_sample();
  data->v_bus = mock_vbus;
  data->temp = 35.0f; // Dummy temp
}

static bool adc_calibrate_offsets(void) {
  printf("[MOCK HAL] ADC Calibrate Offsets\n");
  return true;
}

static bool enc_update(void) {
  // No-op for static mock, simulation handles dynamics
  return mock_encoder_valid;
}

static void enc_get_data(Motor_HAL_EncoderData_t *data) {
  data->position_rad = mock_position;
  data->angle_rad = mock_angle;
  data->velocity_rad = mock_velocity;
  data->elec_angle = mock_electrical_angle;
  data->raw_value = 0;
}

static void enc_set_offset(float offset) {
  printf("[MOCK HAL] Encoder Set Offset: %.3f\n", offset);
}

static void enc_set_pole_pairs(uint8_t pole_pairs) { (void)pole_pairs; }

static void enc_zero_position(void) { mock_position = 0.0f; }

static float enc_get_offset(void) { return 0.0f; }

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
    .set_pole_pairs = enc_set_pole_pairs,
    .zero_position = enc_zero_position,
    .set_offset = enc_set_offset,
    .get_offset = enc_get_offset,
};

// --- Global Handle ---
Motor_HAL_Handle_t g_mock_hal = {
    .pwm = &pwm_impl,
    .adc = &adc_impl,
    .encoder = &enc_impl,
};

Motor_HAL_Handle_t *MockHAL_GetHandle(void) { return &g_mock_hal; }

// ============================================================
// Stubs for symbols referenced by motor.c and fsm.c
// These provide link-time resolution for the test environment.
// ============================================================

// --- Global variables ---
StateMachine g_ds402_state_machine;
uint8_t g_can_id = 1;

// --- HAL PWM stubs (hal_pwm.h) ---
int MHAL_PWM_Enable(void) {
  mock_pwm_enable_count++;
  printf("[STUB] MHAL_PWM_Enable\n");
  return 0;
}
int MHAL_PWM_Disable(void) {
  mock_pwm_disable_count++;
  printf("[STUB] MHAL_PWM_Disable\n");
  return 0;
}
int MHAL_PWM_Brake(void) {
  mock_pwm_brake_count++;
  printf("[STUB] MHAL_PWM_Brake\n");
  return 0;
}
int MHAL_PWM_SetDuty(float ta, float tb, float tc) {
  return (pwm_set_duty(ta, tb, tc), 0);
}

// --- HAL system tick (hal_abstraction.h) ---
uint32_t HAL_GetSystemTick(void) { return 0; }
uint32_t HAL_GetTick(void) { return 0; }
void HAL_EnterCritical(void) {}
void HAL_ExitCritical(void) {}

// --- Link stubs for branches outside this smoke test's exercised path ---
typedef int CalibResult_stub;
void Init_Motor_Calib(void *motor) { (void)motor; }
CalibResult_stub RSLSCalib_Update(void *motor, void *ctx, float period) {
  (void)motor;
  (void)ctx;
  (void)period;
  return 0;
}
CalibResult_stub FluxCalib_Update(void *motor, void *ctx) {
  (void)motor;
  (void)ctx;
  return 0;
}
void Param_ScheduleSave(void) {}
void ADC_SetCurrentOffsets(float ia, float ib, float ic) {
  (void)ia;
  (void)ib;
  (void)ic;
}
void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)code;
  (void)message;
  (void)file;
  (void)line;
}
void Safety_TriggerFault(uint32_t fault_bits, void *motor, void *fsm) {
  (void)fault_bits;
  (void)motor;
  (void)fsm;
}

// --- Safety stubs ---
void Safety_Update_Slow(void *motor, void *fsm) {}
bool Safety_HasActiveFault(void) { return false; }
uint32_t Safety_GetActiveFaultBits(void) { return 0; }

// --- LED stub ---
void RGB_DisplayColorById(uint8_t color_id) { (void)color_id; }
