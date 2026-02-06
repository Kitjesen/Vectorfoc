#include "outer.h"
#include "foc/foc_algorithm.h"
#include "math.h"

/**
 * @brief Outer Control Loops Implementation
 *
 * This file implements low-frequency control loops:
 * - Velocity Control (PID)
 * - Position Control (PID)
 * - Trajectory Tracking
 *
 * It runs at a lower frequency (decimated) compared to the current loop.
 */
bool Control_ShouldRunOuterLoops(const MOTOR_DATA *motor) {
  return (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY &&
          motor->state.Control_Mode <= CONTROL_MODE_POSITION) ||
         (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP &&
          motor->state.Control_Mode <= CONTROL_MODE_POSITION_RAMP);
}

void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  // 原逻辑：4次内环更新一次外环（约 20kHz / 4 = 5kHz）
  if (++ctx->loop_count < 4) {
    return;
  }
  ctx->loop_count = 0;

  if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY &&
      motor->state.Control_Mode <= CONTROL_MODE_POSITION) {
    if (motor->state.Control_Mode == CONTROL_MODE_POSITION) {
      ctx->vel_set = PID_Calc(&motor->PosPID, motor->feedback.position,
                              motor->Controller.input_position);
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.input_velocity,
                           +motor->Controller.input_velocity);
    } else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY) {
      ctx->vel_set = motor->Controller.input_velocity;
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.vel_limit,
                           +motor->Controller.vel_limit);
    }

    motor->algo_input.Iq_ref =
        PID_Calc(&motor->VelPID, motor->feedback.velocity, ctx->vel_set);
    motor->algo_input.Id_ref = 0.0f;
    return;
  }

  // ramp / position_ramp
  if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP &&
      motor->state.Control_Mode <= CONTROL_MODE_POSITION_RAMP) {
    if (motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP) {
      ctx->vel_set = PID_Calc(&motor->PosPID, motor->feedback.position,
                              motor->Controller.pos_setpoint) +
                     motor->Controller.vel_setpoint;
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.input_velocity,
                           +motor->Controller.input_velocity);
    } else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY_RAMP) {
      ctx->vel_set = motor->Controller.vel_setpoint;
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.vel_limit,
                           +motor->Controller.vel_limit);
    }

    motor->feedback.velocity =
        CLAMP(motor->feedback.velocity, -motor->Controller.vel_limit,
              +motor->Controller.vel_limit);
    motor->algo_input.Iq_ref =
        PID_Calc(&motor->VelPID, motor->feedback.velocity, ctx->vel_set) +
        motor->Controller.torque_setpoint;
    motor->algo_input.Id_ref = 0.0f;
    return;
  }
}
