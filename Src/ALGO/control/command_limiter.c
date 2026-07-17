// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "command_limiter.h"
#include "config.h"
#include <math.h>

void ControlCommandLimiter_Reset(MotorControlCtx *context,
                                 const MOTOR_DATA *motor) {
  if (context == NULL || motor == NULL)
    return;

  float current_torque = 0.0f;
  if (isfinite(motor->algo_input.Iq_ref) &&
      isfinite(motor->Controller.torque_const)) {
    current_torque = motor->algo_input.Iq_ref * motor->Controller.torque_const;
  }
  RateLimiter_Reset(&context->velocity_limiter, motor->feedback.velocity);
  RateLimiter_Reset(&context->torque_limiter, current_torque);
  context->limited_velocity = motor->feedback.velocity;
  context->limited_torque = current_torque;
}

void ControlCommandLimiter_Init(MotorControlCtx *context,
                                const MOTOR_DATA *motor) {
  if (context == NULL || motor == NULL)
    return;
  RateLimiter_Init(&context->velocity_limiter,
                   motor->Controller.vel_limit * VELOCITY_ACCEL_MULTIPLIER);
  RateLimiter_Init(&context->torque_limiter,
                   motor->Controller.torque_ramp_rate);
  ControlCommandLimiter_Reset(context, motor);
}

void ControlCommandLimiter_Update(MotorControlCtx *context,
                                  const MOTOR_DATA *motor, float dt) {
  if (context == NULL || motor == NULL)
    return;

  RateLimiter_SetMaxRate(&context->velocity_limiter,
                         motor->Controller.vel_limit *
                             VELOCITY_ACCEL_MULTIPLIER);
  RateLimiter_SetMaxRate(&context->torque_limiter,
                         motor->Controller.torque_ramp_rate);
  context->limited_velocity = RateLimiter_Apply(
      &context->velocity_limiter, motor->Controller.input_velocity, dt);
  context->limited_torque = RateLimiter_Apply(
      &context->torque_limiter, motor->Controller.input_torque, dt);
}
