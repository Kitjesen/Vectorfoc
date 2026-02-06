#include "motor.h"
#include "motor_hal_api.h"
#include "motor_plant.h"
#include <math.h>
#include <stdio.h>


// External Interfaces
void MockHAL_SetNoiseLevel(float sigma);
void MockHAL_SetCurrents(float ia, float ib, float ic);
void MockHAL_SetEncoder(float theta, float vel);
void MockHAL_GetPWM(float *a, float *b, float *c);
Motor_HAL_Handle_t *MockHAL_GetHandle(void);
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

int main() {
  printf("Starting Fault Injection Test...\n");

  // Setup
  MotorPlant_Init(&plant);
  memset(&motor, 0, sizeof(MOTOR_DATA));
  motor.components.hal = MockHAL_GetHandle();

  motor.state.State_Mode = STATE_MODE_RUNNING;
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;
  motor.Controller.vel_setpoint = 20.0f;

  // Params
  motor.parameters.pole_pairs = plant.P;
  motor.parameters.Rs = plant.R;
  motor.parameters.Ls = plant.L;
  motor.parameters.flux = plant.Flux;
  motor.Controller.current_limit = 10.0f;
  motor.Controller.voltage_limit = 24.0f;
  motor.Controller.current_ctrl_p_gain = 5.0f;
  motor.Controller.current_ctrl_i_gain = 50.0f;

  // --- Scenario 1: High Noise ---
  printf("Scenario 1: High Sensor Noise (Sigma = 0.5A)\n");
  MockHAL_SetNoiseLevel(0.5f);

  int stable = 1;
  for (int i = 0; i < 10000; i++) {
    float v_alpha, v_beta;
    GetAppliedVoltage_Impl(24.0f, &v_alpha, &v_beta);
    MotorPlant_Step(&plant, v_alpha, v_beta, 0.0f);

    float ia, ib, ic;
    MotorPlant_GetCurrents(&plant, &ia, &ib, &ic);
    MockHAL_SetCurrents(ia, ib, ic);
    MockHAL_SetEncoder(plant.theta, plant.omega);

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
    motor.feedback.position = enc.angle_rad;
    motor.feedback.velocity = enc.velocity_rad;
    motor.feedback.phase_angle = enc.angle_rad * plant.P;
    motor.algo_input.theta_elec = motor.feedback.phase_angle;

    MotorStateTask(&motor);

    // Stability Check: Velocity shouldn't explode
    if (fabs(plant.omega) > 100.0f) {
      stable = 0;
      printf("FAIL: Instability detected at step %d, vel=%.2f\n", i,
             plant.omega);
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
