// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "feedforward.h"
#include "field_weakening.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK_NEAR(actual, expected, tolerance)                                \
  do {                                                                         \
    float a_ = (actual);                                                       \
    float e_ = (expected);                                                     \
    if (fabsf(a_ - e_) > (tolerance)) {                                        \
      printf("FAIL %s:%d: %.6f != %.6f\n", __FILE__, __LINE__, (double)a_,     \
             (double)e_);                                                      \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static int test_feedforward_does_not_accumulate_command(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;
  motor.Controller.vel_setpoint = 10.0f;
  motor.Controller.input_torque = 3.0f;
  motor.Controller.torque_const = 0.5f;
  Feedforward_Params_t params = {
      .inertia = 0.0f,
      .friction_coeff = 0.1f,
  };

  Feedforward_Reset();
  Feedforward_Update(&motor, &params);
  float first_compensation = Feedforward_GetTorque();
  Feedforward_Update(&motor, &params);

  CHECK_NEAR(motor.Controller.input_torque, 3.0f, 1e-6f);
  CHECK_NEAR(first_compensation, 1.0f, 1e-6f);
  CHECK_NEAR(Feedforward_GetTorque(), 1.0f, 1e-6f);
  CHECK_NEAR(Feedforward_GetCurrent(&motor), 2.0f, 1e-6f);
  return 0;
}

static int test_field_weakening_uses_explicit_dt_without_mutating_base(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.algo_input.Id_ref = 1.5f;
  motor.algo_output.voltage_saturated = true;
  FieldWeakening_Config_t config = {
      .max_weakening_current = 10.0f,
      .start_velocity = 100000.0f,
  };

  FieldWeakening_Reset();
  float compensation_50us = FieldWeakening_Calculate(&motor, &config, 0.00005f);
  CHECK_NEAR(compensation_50us, -0.005f, 1e-6f);
  CHECK_NEAR(motor.algo_input.Id_ref, 1.5f, 1e-6f);

  FieldWeakening_Reset();
  float compensation_200us = FieldWeakening_Calculate(&motor, &config, 0.0002f);
  CHECK_NEAR(compensation_200us, -0.02f, 1e-6f);
  CHECK_NEAR(motor.algo_input.Id_ref, 1.5f, 1e-6f);
  return 0;
}

static int test_feedforward_converts_turn_acceleration_to_radians(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.state.Control_Mode = CONTROL_MODE_VELOCITY;
  Feedforward_Params_t params = {
      .inertia = 0.001f,
      .friction_coeff = 0.0f,
  };

  Feedforward_Reset();
  motor.Controller.vel_setpoint = 0.0f;
  Feedforward_Update(&motor, &params);
  motor.Controller.vel_setpoint = 1.0f;
  Feedforward_Update(&motor, &params);

  /* 1 turn/s change at 5 kHz = 5000 turn/s^2 = 10000*pi rad/s^2. */
  CHECK_NEAR(Feedforward_GetTorque(), 31.4159265f, 1e-4f);
  return 0;
}

int main(void) {
  int failures = 0;
  failures += test_feedforward_does_not_accumulate_command();
  failures += test_field_weakening_uses_explicit_dt_without_mutating_base();
  failures += test_feedforward_converts_turn_acceleration_to_radians();
  if (failures == 0) {
    printf("All control compensation tests passed\n");
  }
  return failures == 0 ? 0 : 1;
}
