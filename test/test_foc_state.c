#include "motor/core/motor.h"
#include "motor_hal_api.h"
#include <assert.h>
#include <stdio.h>


// Mock HAL External
void MockHAL_SetCurrents(float ia, float ib, float ic);
Motor_HAL_Handle_t *MockHAL_GetHandle(void);

// Main Test
int main() {
  printf("Starting FOC State Machine Test...\n");

  // 1. Initialize
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(MOTOR_DATA));
  motor.components.hal = MockHAL_GetHandle();

  // Init Core
  // We might need to call Init_Motor_No_Calib(&motor) if we can link it
  // But for unit test, manual state setup is often safer if dependencies are
  // complex
  motor.state.State_Mode = STATE_MODE_IDLE;
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;

  printf("Initial State: %d (Expected %d)\n", motor.state.State_Mode,
         STATE_MODE_IDLE);
  if (motor.state.State_Mode != STATE_MODE_IDLE)
    return 1;

  // 2. Transition to Running
  // In actual code, this is triggered by CAN command or API.
  // Here we manually force it or assume MotorStateTask handles it if flagged.
  // Let's mimic the API: Motor_SetState(&motor, STATE_MODE_RUNNING);
  // Since we don't have the API linked easily without "motor_api.c", we set
  // manually.
  motor.state.State_Mode = STATE_MODE_RUNNING;
  motor.Controller.vel_setpoint = 10.0f;

  // 3. Step the Task
  // Mock Sensors
  Motor_HAL_SensorData_t sens = {0};
  sens.v_bus = 24.0f;
  motor.components.hal->adc->update(&sens);
  motor.algo_input.Vbus = sens.v_bus;

  MotorStateTask(&motor);

  printf("State after 1 step: %d\n", motor.state.State_Mode);

  // 4. Verify Output
  // If running, we should see non-zero PWM if logic works
  // But Vq might be 0 if P-gain is 0.
  // Let's set some params
  motor.Controller.current_ctrl_p_gain = 1.0f;
  motor.Controller.current_ctrl_i_gain = 1.0f;
  motor.algo_input.Iq_ref =
      1.0f; // Force some simulated ref if outer loop didn't run

  MotorStateTask(&motor);

  // Check algo output
  printf("Algo Vq: %.2f\n", motor.algo_output.Vq);

  // Pass if state is maintained
  if (motor.state.State_Mode == STATE_MODE_RUNNING) {
    printf("State Test: PASS\n");
    return 0;
  } else {
    printf("State Test: FAIL\n");
    return 1;
  }
}
