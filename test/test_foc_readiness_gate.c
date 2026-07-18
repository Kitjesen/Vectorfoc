// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "isr_foc.h"
#include "motor.h"
#include "motor_hal_api.h"
#include "watchdog_supervisor.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static ADC_TypeDef s_adc1_regs;
static ADC_TypeDef s_adc2_regs;
ADC_HandleTypeDef hadc1 = {.Instance = &s_adc1_regs};
ADC_HandleTypeDef hadc2 = {.Instance = &s_adc2_regs};

MOTOR_DATA motor_data;
StateMachine g_ds402_state_machine;

static bool s_ready;
static unsigned s_adc_update_count;
static unsigned s_encoder_update_count;
static unsigned s_encoder_get_count;
static unsigned s_encoder_set_pole_pairs_count;
static unsigned s_safety_fast_count;
static unsigned s_motor_state_count;
static unsigned s_observer_count;
static unsigned s_pwm_disable_count;
static unsigned s_pwm_brake_count;
static unsigned s_fault_count;

bool App_IsFocRuntimeReady(void) { return s_ready; }

static void test_adc_update(Motor_HAL_SensorData_t *data) {
  s_adc_update_count++;
  data->i_a = 1.0f;
  data->i_b = -0.5f;
  data->i_c = -0.5f;
  data->v_bus = 24.0f;
  data->temp = 30.0f;
}

static bool test_encoder_update(void) {
  s_encoder_update_count++;
  return true;
}

static void test_encoder_get_data(Motor_HAL_EncoderData_t *data) {
  s_encoder_get_count++;
  data->position_rad = M_2PI;
  data->velocity_rad = M_2PI * 2.0f;
  data->elec_angle = 0.25f;
}

static void test_encoder_set_pole_pairs(uint8_t pole_pairs) {
  (void)pole_pairs;
  s_encoder_set_pole_pairs_count++;
}

static const Motor_HAL_AdcInterface_t s_adc = {
    .update = test_adc_update,
    .calibrate_offsets = NULL,
};

static const Motor_HAL_EncoderInterface_t s_encoder = {
    .update = test_encoder_update,
    .get_data = test_encoder_get_data,
    .set_pole_pairs = test_encoder_set_pole_pairs,
    .zero_position = NULL,
    .set_offset = NULL,
    .get_offset = NULL,
};

static Motor_HAL_Handle_t s_hal = {
    .pwm = NULL,
    .adc = &s_adc,
    .encoder = &s_encoder,
};

static void reset_fixture(bool ready) {
  memset(&motor_data, 0, sizeof(motor_data));
  memset(&g_ds402_state_machine, 0, sizeof(g_ds402_state_machine));
  s_adc1_regs.JDR1 = 1000u;
  s_adc1_regs.JDR2 = 1100u;
  s_adc1_regs.JDR3 = 1200u;
  s_adc1_regs.JDR4 = 3000u;
  hadc1.ErrorCode = 0u;
  hadc1.Flags = ADC_FLAG_JEOS;
  hadc2.ErrorCode = 0u;
  hadc2.Flags = ADC_FLAG_JEOS;
  motor_data.components.hal = &s_hal;
  motor_data.parameters.pole_pairs = 7u;
  motor_data.state.Control_Mode = CONTROL_MODE_TORQUE;
  s_ready = ready;
  s_adc_update_count = 0u;
  s_encoder_update_count = 0u;
  s_encoder_get_count = 0u;
  s_encoder_set_pole_pairs_count = 0u;
  s_safety_fast_count = 0u;
  s_motor_state_count = 0u;
  s_observer_count = 0u;
  s_pwm_disable_count = 0u;
  s_pwm_brake_count = 0u;
  s_fault_count = 0u;
}

static int test_callback_does_not_touch_runtime_when_not_ready(void) {
  reset_fixture(false);
  uint32_t heartbeat_before = WatchdogSupervisor_GetFOCHeartbeat();

  HAL_ADCEx_InjectedConvCpltCallback(&hadc1);

  if (s_adc_update_count != 0u || s_encoder_update_count != 0u ||
      s_encoder_get_count != 0u || s_safety_fast_count != 0u ||
      s_motor_state_count != 0u || s_observer_count != 0u ||
      s_pwm_disable_count != 0u || s_pwm_brake_count != 0u ||
      s_fault_count != 0u ||
      WatchdogSupervisor_GetFOCHeartbeat() != heartbeat_before) {
    printf("FAIL not-ready ISR touched FOC runtime path\n");
    return 1;
  }
  return 0;
}

static int test_callback_enters_runtime_path_when_ready(void) {
  reset_fixture(true);
  uint32_t heartbeat_before = WatchdogSupervisor_GetFOCHeartbeat();

  HAL_ADCEx_InjectedConvCpltCallback(&hadc1);

  if (s_adc_update_count != 1u || s_encoder_update_count != 1u ||
      s_encoder_get_count != 1u || s_encoder_set_pole_pairs_count != 1u ||
      s_safety_fast_count != 1u || s_motor_state_count != 1u ||
      s_observer_count != 1u || s_pwm_disable_count != 0u ||
      s_pwm_brake_count != 0u || s_fault_count != 0u ||
      WatchdogSupervisor_GetFOCHeartbeat() != heartbeat_before + 1u ||
      motor_data.algo_input.Ia != 1.0f ||
      motor_data.feedback.position != 1.0f ||
      motor_data.feedback.velocity != 2.0f ||
      motor_data.algo_input.theta_elec != 0.25f) {
    printf("FAIL ready ISR did not enter expected runtime path\n");
    return 1;
  }
  return 0;
}

int main(void) {
  int failures = 0;
  failures += test_callback_does_not_touch_runtime_when_not_ready();
  failures += test_callback_enters_runtime_path_when_ready();
  if (failures != 0) {
    printf("%d FOC readiness gate test(s) FAILED\n", failures);
    return 1;
  }
  printf("FOC readiness gate tests passed\n");
  return 0;
}

void Safety_Update_Fast(MOTOR_DATA *motor, StateMachine *fsm) {
  (void)motor;
  (void)fsm;
  s_safety_fast_count++;
}

void Safety_TriggerFault(uint32_t fault_bits, MOTOR_DATA *motor,
                         StateMachine *fsm) {
  (void)fault_bits;
  (void)motor;
  (void)fsm;
  s_fault_count++;
}

int MHAL_PWM_Disable(void) {
  s_pwm_disable_count++;
  return 0;
}

int MHAL_PWM_Brake(void) {
  s_pwm_brake_count++;
  return 0;
}

void StateMachine_Update(StateMachine *sm) { (void)sm; }

void MotorStateTask(MOTOR_DATA *motor) {
  (void)motor;
  s_motor_state_count++;
}

void Motor_API_Feedforward_Update(MOTOR_DATA *motor) { (void)motor; }
void Motor_API_Cogging_Update(MOTOR_DATA *motor) { (void)motor; }

void Motor_API_Observer_Update(MOTOR_DATA *motor) {
  (void)motor;
  s_observer_count++;
}
