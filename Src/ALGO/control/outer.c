#include "outer.h"
#include "config.h"
#include "control/ladrc.h"
#include "foc/foc_algorithm.h"
#include "pid.h"
#include "math.h"
/**
 * @brief Outer Control Loops Implementation
 *
 * This file implements low-frequency control loops:
 * - Velocity Control (PID or LADRC)
 * - Position Control (PID)
 * - Trajectory Tracking
 *
 * It runs at a lower frequency (decimated) compared to the current loop.
 *
 *  motor->ladrc_enable >= 0.5f ，speed/velocity LADRC  PID。
 * LADRC param。
 */
/** outer loopsampleperiod (20kHz / 4 = 5kHz → 0.2ms) */
#define OUTER_LOOP_DT (4.0f * CONTROL_PERIOD_S)

/** 速度反馈低通滤波截止频率 [Hz] */
#define VEL_FEEDBACK_FILTER_FC 100.0f

/** 速度滤波状态 - 每个电机实例应有独立状态，但当前只有单电机 */
/* [FIX] 将静态变量移入函数或使用 motor 结构体存储，避免多电机实例问题 */
/* 当前保持静态变量，但添加注释说明限制 */
static float s_vel_filtered = 0.0f;
static bool s_vel_filter_initialized = false;
/**
 * @brief speed/velocitycalc ( PID  LADRC)
 * @param motor  motor
 * @param vel_ref speed/velocity [turn/s]
 * @param vel_fdb speed/velocityfeedback [turn/s]
 * @return Iq current [A]
 */
static inline float VelLoop_Calc(MOTOR_DATA *motor, float vel_ref,
                                 float vel_fdb) {
  if (motor->ladrc_enable >= 0.5f) {
    /* ── LADRC mode ── */
    /* param，calc LESO gain */
    if (motor->params_updated) {
      LADRC_UpdateGains(&motor->ladrc_state, &motor->ladrc_config);
    }
    return LADRC_Calc(&motor->ladrc_state, &motor->ladrc_config, vel_ref,
                      vel_fdb, motor->ladrc_state.output, OUTER_LOOP_DT);
  }
  /* ──  PID mode ── */
  /* [FIX] Use PID_CalcDt with actual dt instead of PID_Calc with implicit dt=1 */
  return PID_CalcDt(&motor->VelPID, vel_fdb, vel_ref, OUTER_LOOP_DT);
}
bool Control_ShouldRunOuterLoops(const MOTOR_DATA *motor) {
  return (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY &&
          motor->state.Control_Mode <= CONTROL_MODE_POSITION) ||
         (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP &&
          motor->state.Control_Mode <= CONTROL_MODE_POSITION_RAMP);
}
void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  // ：4inner loopupdateouter loop（ 20kHz / 4 = 5kHz）
  if (++ctx->loop_count < 4) {
    return;
  }
  ctx->loop_count = 0;

  // [FIX] 速度反馈低通滤波，减少噪声放大
  if (!s_vel_filter_initialized) {
    s_vel_filtered = motor->feedback.velocity;
    s_vel_filter_initialized = true;
  }
  float alpha_vel = (M_2PI * VEL_FEEDBACK_FILTER_FC * OUTER_LOOP_DT) / 
                    (1.0f + M_2PI * VEL_FEEDBACK_FILTER_FC * OUTER_LOOP_DT);
  s_vel_filtered += alpha_vel * (motor->feedback.velocity - s_vel_filtered);
  float vel_fdb = s_vel_filtered;

  if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY &&
      motor->state.Control_Mode <= CONTROL_MODE_POSITION) {
    if (motor->state.Control_Mode == CONTROL_MODE_POSITION) {
      /* [FIX] Use PID_CalcDt with actual dt for position loop */
      ctx->vel_set = PID_CalcDt(&motor->PosPID, motor->feedback.position,
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
  // ramp / position_ramp
  if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP &&
      motor->state.Control_Mode <= CONTROL_MODE_POSITION_RAMP) {
    if (motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP) {
      /* [FIX] Use PID_CalcDt with actual dt for position ramp loop */
      ctx->vel_set = PID_CalcDt(&motor->PosPID, motor->feedback.position,
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
    return;
  }
}
