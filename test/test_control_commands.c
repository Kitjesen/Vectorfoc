// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "command_limiter.h"
#include "torque_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);              \
      return 1;                                                                \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                \
  do {                                                                         \
    if (fabsf((actual) - (expected)) > (tolerance)) {                          \
      printf("FAIL %s:%d: %.6f != %.6f\n", __FILE__, __LINE__,                 \
             (double)(actual), (double)(expected));                            \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static int test_limiter_preserves_raw_target_and_keeps_ramping(void) {
  MOTOR_DATA motor;
  MotorControlCtx context;
  memset(&motor, 0, sizeof(motor));
  memset(&context, 0, sizeof(context));
  motor.Controller.vel_limit = 10.0f;
  motor.Controller.torque_ramp_rate = 2.0f;
  motor.Controller.input_velocity = 5.0f;
  motor.Controller.input_torque = 1.0f;

  ControlCommandLimiter_Init(&context, &motor);
  float previous_velocity = context.limited_velocity;
  float previous_torque = context.limited_torque;
  for (int i = 0; i < 10; ++i) {
    ControlCommandLimiter_Update(&context, &motor, 0.00005f);
    CHECK(context.limited_velocity > previous_velocity);
    CHECK(context.limited_torque > previous_torque);
    previous_velocity = context.limited_velocity;
    previous_torque = context.limited_torque;
  }

  CHECK_NEAR(motor.Controller.input_velocity, 5.0f, 1e-6f);
  CHECK_NEAR(motor.Controller.input_torque, 1.0f, 1e-6f);
  CHECK(context.limited_velocity < motor.Controller.input_velocity);
  CHECK(context.limited_torque < motor.Controller.input_torque);
  return 0;
}

static int test_mode_entry_resets_to_current_output(void) {
  MOTOR_DATA motor;
  MotorControlCtx context;
  memset(&motor, 0, sizeof(motor));
  memset(&context, 0, sizeof(context));
  motor.Controller.vel_limit = 10.0f;
  motor.Controller.torque_ramp_rate = 2.0f;
  motor.Controller.torque_const = 0.5f;
  motor.feedback.velocity = -2.0f;
  motor.algo_input.Iq_ref = 4.0f;

  ControlCommandLimiter_Init(&context, &motor);
  CHECK_NEAR(context.limited_velocity, -2.0f, 1e-6f);
  CHECK_NEAR(context.limited_torque, 2.0f, 1e-6f);
  return 0;
}

static int test_torque_conversion_rejects_invalid_constant(void) {
  float current = 123.0f;
  CHECK(Control_TorqueToCurrent(2.0f, 0.5f, &current));
  CHECK_NEAR(current, 4.0f, 1e-6f);

  CHECK(!Control_TorqueToCurrent(2.0f, 0.0f, &current));
  CHECK_NEAR(current, 0.0f, 1e-6f);
  CHECK(!Control_TorqueToCurrent(2.0f, NAN, &current));
  CHECK(!Control_TorqueToCurrent(NAN, 0.5f, &current));
  return 0;
}

static int test_inertia_and_position_limit_units(void) {
  CHECK_NEAR(Control_InertiaTorque(0.001f, 5000.0f), 31.4159265f, 1e-4f);
  CHECK_NEAR(Control_PositionVelocityLimit(-3.0f, 2.0f), 2.0f, 1e-6f);
  CHECK_NEAR(Control_PositionVelocityLimit(1.0f, 2.0f), 1.0f, 1e-6f);
  return 0;
}

int main(void) {
  int failures = 0;
  failures += test_limiter_preserves_raw_target_and_keeps_ramping();
  failures += test_mode_entry_resets_to_current_output();
  failures += test_torque_conversion_rejects_invalid_constant();
  failures += test_inertia_and_position_limit_units();
  if (failures == 0)
    printf("All control command tests passed\n");
  return failures == 0 ? 0 : 1;
}
