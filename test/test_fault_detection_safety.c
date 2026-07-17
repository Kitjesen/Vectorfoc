// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "fault_detection.h"
#include "motor.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_tick_ms;
static float s_velocity;

#undef HAL_GetSystemTick
uint32_t HAL_GetSystemTick(void) { return s_tick_ms; }
float MHAL_Encoder_GetVelocity(void) { return s_velocity; }

static MOTOR_DATA make_motor(CONTROL_MODE mode) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.state.Control_Mode = mode;
  motor.algo_input.Vbus = 24.0f;
  motor.feedback.temperature = 25.0f;
  return motor;
}

static void test_open_loop_keeps_electrical_and_thermal_protection(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_OPEN);
  Detection_Init(NULL);

  motor.algo_input.Ia = 100.0f;
  assert((Detection_Check_Fast(&motor) & FAULT_CURRENT_A) != 0u);

  Detection_Init(NULL);
  motor.algo_input.Ia = 0.0f;
  motor.algo_input.Vbus = 70.0f;
  assert((Detection_Check_Slow(&motor) & FAULT_OVER_VOLTAGE) != 0u);

  Detection_Init(NULL);
  motor.algo_input.Vbus = 24.0f;
  motor.feedback.temperature = 160.0f;
  assert((Detection_Check_Slow(&motor) & FAULT_OVER_TEMP) != 0u);
}

static void test_non_finite_sensor_values_fail_safe(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);

  motor.algo_input.Ia = NAN;
  motor.algo_input.Ib = INFINITY;
  motor.algo_input.Ic = -INFINITY;
  uint32_t fault = Detection_Check_Fast(&motor);
  assert((fault & FAULT_OVER_CURRENT) == FAULT_OVER_CURRENT);

  Detection_Init(NULL);
  motor.algo_input.Ia = motor.algo_input.Ib = motor.algo_input.Ic = 0.0f;
  motor.algo_input.Vbus = NAN;
  fault = Detection_Check_Slow(&motor);
  assert((fault & (FAULT_OVER_VOLTAGE | FAULT_UNDER_VOLTAGE)) != 0u);

  Detection_Init(NULL);
  motor.algo_input.Vbus = 24.0f;
  motor.feedback.temperature = NAN;
  assert((Detection_Check_Slow(&motor) & FAULT_OVER_TEMP) != 0u);
}

static void test_stall_timeout_uses_elapsed_milliseconds(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_can_timeout = false;
  config->stall_current_threshold = 5.0f;
  config->stall_velocity_threshold = 1.0f;
  config->stall_detect_time_ms = 500u;
  motor.algo_input.Ia = 10.0f;
  motor.algo_input.Ib = 10.0f;
  motor.algo_input.Ic = 10.0f;
  s_velocity = 0.0f;

  s_tick_ms = 1000u;
  assert((Detection_Check_Slow(&motor) & FAULT_STALL_OVERLOAD) == 0u);
  s_tick_ms = 1499u;
  assert((Detection_Check_Slow(&motor) & FAULT_STALL_OVERLOAD) == 0u);
  s_tick_ms = 1500u;
  assert((Detection_Check_Slow(&motor) & FAULT_STALL_OVERLOAD) != 0u);
}

static void test_can_timeout_configuration_and_recovery(void) {
  MOTOR_DATA motor = make_motor(CONTROL_MODE_TORQUE);
  Detection_Init(NULL);
  DetectionConfig *config = Detection_GetConfig();
  config->enable_voltage_protection = false;
  config->enable_temp_protection = false;
  config->enable_stall_protection = false;

  s_tick_ms = 2000u;
  Detection_SetCANTimeout(100u);
  assert(config->enable_can_timeout);
  assert(config->can_timeout_ms == 100u);
  s_tick_ms = 2100u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);
  s_tick_ms = 2101u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) != 0u);

  Detection_FeedWatchdog(s_tick_ms);
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);

  Detection_SetCANTimeout(0u);
  assert(!config->enable_can_timeout);
  s_tick_ms += 10000u;
  assert((Detection_Check_Slow(&motor) & FAULT_CAN_TIMEOUT) == 0u);
}

int main(void) {
  test_open_loop_keeps_electrical_and_thermal_protection();
  test_non_finite_sensor_values_fail_safe();
  test_stall_timeout_uses_elapsed_milliseconds();
  test_can_timeout_configuration_and_recovery();
  puts("Fault detection safety tests: PASS");
  return 0;
}
