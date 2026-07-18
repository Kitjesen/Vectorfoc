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


// External access to mock
void MockHAL_SetCurrents(float ia, float ib, float ic);
void MockHAL_SetEncoder(float position, float angle, float velocity,
                        float electrical_angle);
void MockHAL_GetPWM(float *a, float *b, float *c);
Motor_HAL_Handle_t *MockHAL_GetHandle(void);

// FSM instance defined in mock_hal.c
extern StateMachine g_ds402_state_machine;

// External access to Motor Task functions if not exposed
// Assuming we link against MotorStateTask
extern void MotorStateTask(MOTOR_DATA *motor);

// Simple global for simulation
static MotorPlant_t plant;
static MOTOR_DATA motor;
static ADC_TypeDef adc1_regs;
static ADC_TypeDef adc2_regs;
ADC_HandleTypeDef hadc1 = {.Instance = &adc1_regs};
ADC_HandleTypeDef hadc2 = {.Instance = &adc2_regs};

static int Fail(const char *message) {
  printf("FAIL: %s\n", message);
  return 1;
}

static int IsBoundedFinite(const char *name, float value, float limit) {
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

static int DutyIsValid(const char *name, float duty) {
  if (!isfinite(duty) || duty < 0.0f || duty > 1.0f) {
    printf("FAIL: %s duty out of range: %.6f\n", name, duty);
    return 0;
  }
  return 1;
}

// Helper to convert Duty Cycle to Voltage
void GetAppliedVoltage(float v_bus, float *v_alpha, float *v_beta) {
  float da, db, dc;
  MockHAL_GetPWM(&da, &db, &dc);

  float va = da * v_bus;
  float vb = db * v_bus;
  float vc = dc * v_bus;

  // Clarke Transform for Voltages
  // Valpha = 2/3 * (Va - 0.5Vb - 0.5Vc)
  // Vbeta  = 2/3 * (sqrt(3)/2 * Vb - sqrt(3)/2 * Vc) => 1/sqrt(3) * (Vb - Vc)

  *v_alpha = 0.6666f * (va - 0.5f * vb - 0.5f * vc);
  *v_beta = 0.57735f * (vb - vc);
}

int main() {
  printf("Starting Closed Loop Simulation...\n");

  // 1. Init Plant
  MotorPlant_Init(&plant);

  // 2. Init Motor Controller
  memset(&motor, 0, sizeof(MOTOR_DATA));

  // Wiring HAL
  motor.components.hal = MockHAL_GetHandle();

  // Params (Match Plant where possible)
  motor.parameters.pole_pairs = plant.P;
  motor.parameters.Rs = plant.R;
  motor.parameters.Ls = plant.L;
  motor.parameters.flux = plant.Flux; // Wb

  // Control Config
  motor.Controller.current_limit = 10.0f;
  motor.Controller.voltage_limit = 24.0f;
  motor.Controller.vel_limit = 20.0f;
  motor.Controller.current_ctrl_p_gain = 10.0f;  // Roughly L * BW
  motor.Controller.current_ctrl_i_gain = 100.0f; // Roughly R * BW
  motor.IdPID.Kp = motor.Controller.current_ctrl_p_gain;
  motor.IdPID.Ki = motor.Controller.current_ctrl_i_gain;
  motor.IqPID.Kp = motor.Controller.current_ctrl_p_gain;
  motor.IqPID.Ki = motor.Controller.current_ctrl_i_gain;
  motor.VelPID.Kp = 0.05f;
  motor.VelPID.Ki = 1.0f;
  motor.VelPID.max_out = motor.Controller.current_limit;
  motor.VelPID.max_iout = motor.Controller.current_limit;
  motor.params_updated = true;

  // Init State - set FSM to OPERATION_ENABLED so MotorStateTask maps to RUNNING
  StateMachine_Init(&g_ds402_state_machine);
  g_ds402_state_machine.current_state = STATE_OPERATION_ENABLED;
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;

  // Target
  motor.Controller.input_velocity = 50.0f / M_2PI; // 50 rad/s
  motor.Controller.vel_setpoint = motor.Controller.input_velocity;

  // 3. Open Log
  FILE *f = fopen("sim_response.csv", "w");
  if (f == NULL) {
    return Fail("could not open sim_response.csv");
  }
  fprintf(f, "Time,RefVel,ActVel,Iq,Id,Ialpha_Sim,Ibeta_Sim\n");

  // 4. Run Loop
  for (int i = 0; i < 4000; i++) { // 0.2s at 20kHz
    float t = i * plant.dt;

    // --- Step Plant ---
    float v_alpha, v_beta;
    GetAppliedVoltage(24.0f, &v_alpha, &v_beta);
    MotorPlant_Step(&plant, v_alpha, v_beta, 0.0f); // 0 Load

    if (!IsBoundedFinite("plant omega", plant.omega, 500.0f) ||
        !IsBoundedFinite("plant i_alpha", plant.i_alpha, 80.0f) ||
        !IsBoundedFinite("plant i_beta", plant.i_beta, 80.0f)) {
      fclose(f);
      return 1;
    }

    // --- Feedback to Sensor ---
    float ia, ib, ic;
    MotorPlant_GetCurrents(&plant, &ia, &ib, &ic);
    MockHAL_SetCurrents(ia, ib, ic);
    MockHAL_SetEncoder(plant.position, plant.theta, plant.omega,
                       fmodf(plant.theta * (float)plant.P, M_2PI));

    // --- Step Controller (Simulate ADC Callback) ---
    // Manually trigger the update sequence
    // 1. Update Sensors
    Motor_HAL_SensorData_t sens;
    motor.components.hal->adc->update(&sens);
    motor.algo_input.Ia = sens.i_a;
    motor.algo_input.Ib = sens.i_b;
    motor.algo_input.Ic = sens.i_c;
    motor.algo_input.Vbus = sens.v_bus;

    // 2. Update Encoder
    Motor_HAL_EncoderData_t enc;
    motor.components.hal->encoder->get_data(&enc);
    motor.feedback.position = enc.position_rad / M_2PI;
    motor.feedback.velocity = enc.velocity_rad / M_2PI;
    motor.feedback.phase_angle = enc.elec_angle;
    motor.algo_input.theta_elec = motor.feedback.phase_angle;

    // 3. Run FOC Logic
    MotorStateTask(&motor);

    float da, db, dc;
    MockHAL_GetPWM(&da, &db, &dc);
    if (!DutyIsValid("phase A", da) || !DutyIsValid("phase B", db) ||
        !DutyIsValid("phase C", dc) ||
        !IsBoundedFinite("Iq output", motor.algo_output.Iq, 10.5f) ||
        !IsBoundedFinite("Id output", motor.algo_output.Id, 10.5f)) {
      fclose(f);
      return 1;
    }

    // --- Log ---
    if (i % 10 == 0) { // Log every 10th step
      fprintf(f, "%.4f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\n", t,
              motor.Controller.vel_setpoint, plant.omega, motor.algo_output.Iq,
              motor.algo_output.Id, plant.i_alpha, plant.i_beta);
    }
  }

  fclose(f);
  float final_error = fabsf(plant.omega - 50.0f);
  if (final_error > 35.0f) {
    printf("FAIL: final velocity error too high: %.6f rad/s\n", final_error);
    return 1;
  }

  printf("Simulation Complete. Data saved to sim_response.csv\n");
  return 0;
}
