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

#include "motor.h"
#include "motor_hal_api.h"
#include "fsm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


// Mock HAL External
void MockHAL_SetCurrents(float ia, float ib, float ic);
void MockHAL_SetEncoder(float position, float angle, float velocity,
                        float electrical_angle);
void MockHAL_GetPWM(float *a, float *b, float *c);
void MockHAL_Reset(void);
int MockHAL_GetPwmSetDutyCount(void);
int MockHAL_GetPwmEnableCount(void);
int MockHAL_GetPwmDisableCount(void);
int MockHAL_GetPwmBrakeCount(void);
Motor_HAL_Handle_t *MockHAL_GetHandle(void);

// FSM instance defined in mock_hal.c
extern StateMachine g_ds402_state_machine;

static ADC_TypeDef s_adc1_regs;
static ADC_TypeDef s_adc2_regs;
ADC_HandleTypeDef hadc1 = {.Instance = &s_adc1_regs};
ADC_HandleTypeDef hadc2 = {.Instance = &s_adc2_regs};

static void init_motor_fixture(MOTOR_DATA *motor) {
  memset(motor, 0, sizeof(MOTOR_DATA));
  motor->components.hal = MockHAL_GetHandle();
  motor->parameters.Rs = 0.2f;
  motor->parameters.Ls = 0.0003f;
  motor->parameters.flux = 0.01f;
  motor->parameters.pole_pairs = 7;
  motor->Controller.voltage_limit = 24.0f;
  motor->Controller.current_limit = 8.0f;
  motor->Controller.torque_limit = 1.0f;
  motor->Controller.torque_const = 0.1f;
  motor->Controller.torque_ramp_rate = 10000.0f;
  motor->Controller.vel_limit = 100.0f;
  motor->Controller.current_ctrl_p_gain = 0.3f;
  motor->Controller.current_ctrl_i_gain = 40.0f;
  motor->state.Control_Mode = CONTROL_MODE_TORQUE;
  motor->algo_input.Vbus = 24.0f;
  motor->algo_input.theta_elec = 0.0f;
  motor->params_updated = true;
}

static int duty_is_bounded(float duty) {
  return isfinite(duty) && duty >= 0.0f && duty <= 1.0f;
}

static int test_operation_enabled_runs_control_and_requests_power(void) {
  MOTOR_DATA motor;
  MockHAL_Reset();
  init_motor_fixture(&motor);
  StateMachine_Init(&g_ds402_state_machine);
  g_ds402_state_machine.current_state = STATE_OPERATION_ENABLED;
  motor.Controller.input_torque = 0.2f;
  MockHAL_SetCurrents(0.0f, 0.0f, 0.0f);
  MockHAL_SetEncoder(0.0f, 0.0f, 0.0f, 0.0f);

  MotorStateTask(&motor);

  float da, db, dc;
  MockHAL_GetPWM(&da, &db, &dc);
  if (motor.state.State_Mode != STATE_MODE_RUNNING ||
      !g_ds402_state_machine.operation_power_enabled ||
      MockHAL_GetPwmSetDutyCount() != 1 || MockHAL_GetPwmEnableCount() != 1 ||
      MockHAL_GetPwmDisableCount() != 0 || MockHAL_GetPwmBrakeCount() != 0 ||
      !motor.algo_input.enabled || !duty_is_bounded(da) ||
      !duty_is_bounded(db) || !duty_is_bounded(dc)) {
    printf("FAIL operation-enabled control path did not produce safe PWM\n");
    return 1;
  }
  return 0;
}

static int test_fault_state_disables_pwm_without_running_control(void) {
  MOTOR_DATA motor;
  MockHAL_Reset();
  init_motor_fixture(&motor);
  StateMachine_Init(&g_ds402_state_machine);
  g_ds402_state_machine.current_state = STATE_FAULT;
  motor.Controller.input_torque = 0.2f;

  MotorStateTask(&motor);

  if (motor.state.State_Mode != STATE_MODE_GUARD ||
      MockHAL_GetPwmDisableCount() != 1 ||
      MockHAL_GetPwmSetDutyCount() != 0 ||
      g_ds402_state_machine.operation_power_enabled) {
    printf("FAIL fault state did not remain de-energized\n");
    return 1;
  }
  return 0;
}

static int test_current_calibration_reads_typed_adc_jdr_registers(void) {
  MOTOR_DATA motor;
  MockHAL_Reset();
  init_motor_fixture(&motor);
  StateMachine_Init(&g_ds402_state_machine);
  g_ds402_state_machine.current_state = STATE_CALIBRATING;
  motor.calib_type_requested = 3;
  motor.state.Sub_State = CURRENT_CALIBRATING;
  s_adc1_regs.JDR1 = 3000u;
  s_adc1_regs.JDR2 = 2100u;
  s_adc1_regs.JDR3 = 2200u;

  MotorStateTask(&motor);
  if (motor.state.State_Mode != STATE_MODE_DETECTING ||
      !motor.calib_ctx.current.is_initialized ||
      MockHAL_GetPwmBrakeCount() != 1) {
    printf("FAIL current calibration did not initialize safely\n");
    return 1;
  }

  MotorStateTask(&motor);

  if (motor.state.State_Mode != STATE_MODE_DETECTING ||
      motor.calib_ctx.current.loop_count != 1u ||
      fabsf(motor.calib_ctx.current.offset_sum_a - 2200.0f) > 0.5f ||
      fabsf(motor.calib_ctx.current.offset_sum_b - 2100.0f) > 0.5f ||
      fabsf(motor.calib_ctx.current.offset_sum_c - 3000.0f) > 0.5f) {
    printf("FAIL current calibration did not read typed ADC JDR registers "
           "(state=%d loop=%u sums=%.1f/%.1f/%.1f)\n",
           motor.state.State_Mode, motor.calib_ctx.current.loop_count,
           motor.calib_ctx.current.offset_sum_a,
           motor.calib_ctx.current.offset_sum_b,
           motor.calib_ctx.current.offset_sum_c);
    return 1;
  }
  return 0;
}

int main(void) {
  int failures = 0;
  failures += test_operation_enabled_runs_control_and_requests_power();
  failures += test_fault_state_disables_pwm_without_running_control();
  failures += test_current_calibration_reads_typed_adc_jdr_registers();
  if (failures != 0) {
    printf("%d FOC state test(s) FAILED\n", failures);
    return 1;
  }
  printf("FOC state machine integration tests passed\n");
  return 0;
}
