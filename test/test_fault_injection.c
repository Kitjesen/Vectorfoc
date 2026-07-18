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
#include "motor_plant.h"
#include "fsm.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>


// External Interfaces
void MockHAL_SetNoiseLevel(float sigma);
void MockHAL_SetCurrents(float ia, float ib, float ic);
void MockHAL_SetEncoder(float position, float angle, float velocity,
                        float electrical_angle);
void MockHAL_GetPWM(float *a, float *b, float *c);
Motor_HAL_Handle_t *MockHAL_GetHandle(void);

// FSM instance defined in mock_hal.c
extern StateMachine g_ds402_state_machine;
extern void MotorStateTask(MOTOR_DATA *motor);

// Helper from closed loop test
void GetAppliedVoltage(float v_bus, float *v_alpha, float *v_beta);
// Note: We need to copy-paste or link this helper.
// For simplicity, re-implementing briefly:
void GetAppliedVoltage_Impl(float v_bus, float *v_alpha, float *v_beta) {
  float da, db, dc;
  MockHAL_GetPWM(&da, &db, &dc);
  *v_alpha = 0.6666f * (da * v_bus - 0.5f * db * v_bus - 0.5f * dc * v_bus);
  *v_beta = 0.57735f * (db * v_bus - dc * v_bus);
}

static MotorPlant_t plant;
static MOTOR_DATA motor;
static ADC_TypeDef adc1_regs;
static ADC_TypeDef adc2_regs;
ADC_HandleTypeDef hadc1 = {.Instance = &adc1_regs};
ADC_HandleTypeDef hadc2 = {.Instance = &adc2_regs};

static int CheckBoundedFinite(const char *name, float value, float limit) {
  if (!isfinite(value)) {
    printf("FAIL: %s is not finite: %.6f\n", name, value);
    return 0;
  }
  if (fabsf(value) > limit) {
    printf("FAIL: %s out of bounds: %.6f > %.6f\n", name, value, limit);
    return 0;
  }
  return 1;
}

static int CheckDuty(const char *name, float duty) {
  if (!isfinite(duty) || duty < 0.0f || duty > 1.0f) {
    printf("FAIL: %s duty out of range: %.6f\n", name, duty);
    return 0;
  }
  return 1;
}

int main() {
  printf("Starting Fault Injection Test...\n");

  // Setup
  MotorPlant_Init(&plant);
  memset(&motor, 0, sizeof(MOTOR_DATA));
  motor.components.hal = MockHAL_GetHandle();

  StateMachine_Init(&g_ds402_state_machine);
  g_ds402_state_machine.current_state = STATE_OPERATION_ENABLED;
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;
  motor.Controller.input_velocity = 20.0f / M_2PI;
  motor.Controller.vel_setpoint = motor.Controller.input_velocity;

  // Params
  motor.parameters.pole_pairs = plant.P;
  motor.parameters.Rs = plant.R;
  motor.parameters.Ls = plant.L;
  motor.parameters.flux = plant.Flux;
  motor.Controller.current_limit = 10.0f;
  motor.Controller.voltage_limit = 24.0f;
  motor.Controller.vel_limit = 20.0f;
  motor.Controller.current_ctrl_p_gain = 5.0f;
  motor.Controller.current_ctrl_i_gain = 50.0f;
  motor.IdPID.Kp = motor.Controller.current_ctrl_p_gain;
  motor.IdPID.Ki = motor.Controller.current_ctrl_i_gain;
  motor.IqPID.Kp = motor.Controller.current_ctrl_p_gain;
  motor.IqPID.Ki = motor.Controller.current_ctrl_i_gain;
  motor.VelPID.Kp = 0.05f;
  motor.VelPID.Ki = 1.0f;
  motor.VelPID.max_out = motor.Controller.current_limit;
  motor.VelPID.max_iout = motor.Controller.current_limit;
  motor.params_updated = true;

  // --- Scenario 1: High Noise ---
  printf("Scenario 1: High Sensor Noise (Sigma = 0.5A)\n");
  MockHAL_SetNoiseLevel(0.5f);

  int stable = 1;
  for (int i = 0; i < 10000; i++) {
    float v_alpha, v_beta;
    GetAppliedVoltage_Impl(24.0f, &v_alpha, &v_beta);
    MotorPlant_Step(&plant, v_alpha, v_beta, 0.0f);
    if (!CheckBoundedFinite("plant omega", plant.omega, 100.0f) ||
        !CheckBoundedFinite("plant i_alpha", plant.i_alpha, 80.0f) ||
        !CheckBoundedFinite("plant i_beta", plant.i_beta, 80.0f)) {
      stable = 0;
      break;
    }

    float ia, ib, ic;
    MotorPlant_GetCurrents(&plant, &ia, &ib, &ic);
    MockHAL_SetCurrents(ia, ib, ic);
    MockHAL_SetEncoder(plant.position, plant.theta, plant.omega,
                       fmodf(plant.theta * (float)plant.P, M_2PI));

    // Update
    Motor_HAL_SensorData_t sens = {0};
    motor.components.hal->adc->update(&sens);
    motor.algo_input.Ia = sens.i_a;
    motor.algo_input.Ib = sens.i_b;
    motor.algo_input.Ic = sens.i_c;
    motor.algo_input.Vbus = sens.v_bus;

    // Update Enc
    Motor_HAL_EncoderData_t enc = {0};
    motor.components.hal->encoder->get_data(&enc);
    motor.feedback.position = enc.position_rad / M_2PI;
    motor.feedback.velocity = enc.velocity_rad / M_2PI;
    motor.feedback.phase_angle = enc.elec_angle;
    motor.algo_input.theta_elec = motor.feedback.phase_angle;

    MotorStateTask(&motor);

    float da, db, dc;
    MockHAL_GetPWM(&da, &db, &dc);
    if (!CheckDuty("phase A", da) || !CheckDuty("phase B", db) ||
        !CheckDuty("phase C", dc) ||
        !CheckBoundedFinite("Iq output", motor.algo_output.Iq, 10.5f) ||
        !CheckBoundedFinite("Id output", motor.algo_output.Id, 10.5f)) {
      stable = 0;
      printf("FAIL: instability detected at step %d\n", i);
      break;
    }
  }

  if (stable)
    printf("PASS: System stable under noise.\n");

  // --- Scenario 2: Phase Loss ---
  // Not easy to do without modding the plant to infinite resistance.
  // We can simulate it by forcing the current readback to 0 for one phase?
  // Or in the plant, forces Ia calculation?
  // Let's skip complex phase loss and stick to Noise Test for now.

  return stable ? 0 : 1;
}
