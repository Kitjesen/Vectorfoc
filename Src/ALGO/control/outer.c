#include "outer.h"
#include "config.h"
#include "control/ladrc.h"
#include "foc/foc_algorithm.h"
#include "math.h"
#include "pid.h"

#define OUTER_LOOP_DT (4.0f * CONTROL_PERIOD_S)
#define VEL_FEEDBACK_FILTER_FC 100.0f

static float VelLoop_Calc(MOTOR_DATA *motor, float vel_ref, float vel_fdb) {
  if (motor->ladrc_enable >= 0.5f) {
    if (motor->params_updated) {
      LADRC_UpdateGains(&motor->ladrc_state, &motor->ladrc_config);
    }

    return LADRC_Calc(&motor->ladrc_state, &motor->ladrc_config, vel_ref,
                      vel_fdb, motor->ladrc_state.output, OUTER_LOOP_DT);
  }

  return PID_CalcDt(&motor->VelPID, vel_fdb, vel_ref, OUTER_LOOP_DT);
}

bool Control_ShouldRunOuterLoops(const MOTOR_DATA *motor) {
  return (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY &&
          motor->state.Control_Mode <= CONTROL_MODE_POSITION) ||
         (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP &&
          motor->state.Control_Mode <= CONTROL_MODE_POSITION_RAMP);
}

void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  if (++ctx->loop_count < 4) {
    return;
  }
  ctx->loop_count = 0;

  if (!ctx->vel_filter_initialized) {
    ctx->vel_feedback_filtered = motor->feedback.velocity;
    ctx->vel_filter_initialized = true;
  }

  float alpha_vel = (M_2PI * VEL_FEEDBACK_FILTER_FC * OUTER_LOOP_DT) /
                    (1.0f + M_2PI * VEL_FEEDBACK_FILTER_FC * OUTER_LOOP_DT);
  ctx->vel_feedback_filtered +=
      alpha_vel * (motor->feedback.velocity - ctx->vel_feedback_filtered);
  float vel_fdb = ctx->vel_feedback_filtered;

  if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY &&
      motor->state.Control_Mode <= CONTROL_MODE_POSITION) {
    if (motor->state.Control_Mode == CONTROL_MODE_POSITION) {
      ctx->vel_set =
          PID_CalcDt(&motor->PosPID, motor->feedback.position,
                     motor->Controller.input_position, OUTER_LOOP_DT);
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.input_velocity,
                           +motor->Controller.input_velocity);
    } else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY) {
      ctx->vel_set = motor->Controller.input_velocity;
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.vel_limit,
                           +motor->Controller.vel_limit);
    }

    motor->algo_input.Iq_ref = VelLoop_Calc(motor, ctx->vel_set, vel_fdb);
    motor->algo_input.Id_ref = 0.0f;
    return;
  }

  if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP &&
      motor->state.Control_Mode <= CONTROL_MODE_POSITION_RAMP) {
    if (motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP) {
      ctx->vel_set =
          PID_CalcDt(&motor->PosPID, motor->feedback.position,
                     motor->Controller.pos_setpoint, OUTER_LOOP_DT) +
          motor->Controller.vel_setpoint;
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.input_velocity,
                           +motor->Controller.input_velocity);
    } else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY_RAMP) {
      ctx->vel_set = motor->Controller.vel_setpoint;
      ctx->vel_set = CLAMP(ctx->vel_set, -motor->Controller.vel_limit,
                           +motor->Controller.vel_limit);
    }

    vel_fdb = CLAMP(vel_fdb, -motor->Controller.vel_limit,
                    +motor->Controller.vel_limit);
    motor->algo_input.Iq_ref =
        VelLoop_Calc(motor, ctx->vel_set, vel_fdb) +
        motor->Controller.torque_setpoint;
    motor->algo_input.Id_ref = 0.0f;
  }
}
