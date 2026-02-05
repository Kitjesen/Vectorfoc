#include "motor/core/motor.h"
#include "motor_hal_api.h"
#include "motor_plant.h"
#include <stdio.h>


// External access to mock
void MockHAL_SetCurrents(float ia, float ib, float ic);
void MockHAL_SetEncoder(float theta, float vel);
void MockHAL_GetPWM(float *a, float *b, float *c);
Motor_HAL_Handle_t *MockHAL_GetHandle(void);

// External access to Motor Task functions if not exposed
// Assuming we link against MotorStateTask
extern void MotorStateTask(MOTOR_DATA *motor);

// Simple global for simulation
static MotorPlant_t plant;
static MOTOR_DATA motor;

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
  motor.Controller.current_ctrl_p_gain = 10.0f;  // Roughly L * BW
  motor.Controller.current_ctrl_i_gain = 100.0f; // Roughly R * BW

  // Init State
  motor.state.State_Mode = STATE_MODE_RUNNING; // Skip calibration for this test
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;

  // Target
  motor.Controller.vel_setpoint = 50.0f; // 50 rad/s

  // 3. Open Log
  FILE *f = fopen("sim_response.csv", "w");
  fprintf(f, "Time,RefVel,ActVel,Iq,Id,Ialpha_Sim,Ibeta_Sim\n");

  // 4. Run Loop
  for (int i = 0; i < 4000; i++) { // 0.2s at 20kHz
    float t = i * plant.dt;

    // --- Step Plant ---
    float v_alpha, v_beta;
    GetAppliedVoltage(24.0f, &v_alpha, &v_beta);
    MotorPlant_Step(&plant, v_alpha, v_beta, 0.0f); // 0 Load

    // --- Feedback to Sensor ---
    float ia, ib, ic;
    MotorPlant_GetCurrents(&plant, &ia, &ib, &ic);
    MockHAL_SetCurrents(ia, ib, ic);
    MockHAL_SetEncoder(plant.theta, plant.omega);

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
    motor.feedback.position = enc.angle_rad;
    motor.feedback.velocity = enc.velocity_rad;
    motor.feedback.phase_angle = enc.angle_rad * plant.P; // Elec angle
    motor.algo_input.theta_elec = motor.feedback.phase_angle;

    // 3. Run FOC Logic
    MotorStateTask(&motor);

    // --- Log ---
    if (i % 10 == 0) { // Log every 10th step
      fprintf(f, "%.4f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\n", t,
              motor.Controller.vel_setpoint, plant.omega, motor.algo_output.Iq,
              motor.algo_output.Id, plant.i_alpha, plant.i_beta);
    }
  }

  fclose(f);
  printf("Simulation Complete. Data saved to sim_response.csv\n");
  return 0;
}
