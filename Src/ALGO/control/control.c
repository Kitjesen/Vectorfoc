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

#include "control.h"
#include "context.h"
#include "inner.h"
#include "outer.h"
#include "impl.h"
#include "config.h"
#include "foc/foc_algorithm.h"
#include "trajectory/rate_limiter.h"
#include "error_manager.h"
#include "error_types.h"
// Static Context
static MotorControlCtx s_ctx;
// Rate Limiters
static RateLimiterTypeDef s_vel_limiter;    // Velocity Rate Limiter
static RateLimiterTypeDef s_torque_limiter; // Torque Rate Limiter
static bool s_limiters_initialized = false;
void Control_Init(MOTOR_DATA *motor) {
  if (s_limiters_initialized) {
    return;
  }
  // speed/velocity: speed/velocity = vel_limit * VELOCITY_ACCEL_MULTIPLIER
  RateLimiter_Init(&s_vel_limiter,
                   motor->Controller.vel_limit * VELOCITY_ACCEL_MULTIPLIER);
  // : config
  RateLimiter_Init(&s_torque_limiter, motor->Controller.torque_ramp_rate);
  // initFOCstate ( MOTOR_DATA )
  FOC_Algorithm_InitState(&motor->algo_state);
  s_limiters_initialized = true;
}
void MotorControl_Run(MOTOR_DATA *motor) {
  //
  motor->Controller.input_velocity = RateLimiter_Apply(
      &s_vel_limiter, motor->Controller.input_velocity, CURRENT_MEASURE_PERIOD);
  motor->Controller.input_torque =
      RateLimiter_Apply(&s_torque_limiter, motor->Controller.input_torque,
                        CURRENT_MEASURE_PERIOD);
  // mode: limit, ,
  ControlImpl_SetPidLimits(motor);
  switch (motor->state.Control_Mode) {
  // open loop
  case CONTROL_MODE_OPEN:
    ControlImpl_Open(motor);
    break;
  //
  case CONTROL_MODE_TORQUE:
    ControlImpl_Torque(motor, &s_ctx);
    break;
  // speed/velocity
  case CONTROL_MODE_VELOCITY:
    ControlImpl_Velocity(motor);
    break;
  // position
  case CONTROL_MODE_POSITION:
    ControlImpl_Position(motor);
    break;
  // speed/velocity
  case CONTROL_MODE_VELOCITY_RAMP:
    ControlImpl_VelocityRamp(motor);
    break;
  // position
  case CONTROL_MODE_POSITION_RAMP:
    ControlImpl_PositionRamp(motor, &s_ctx);
    break;
  // MIT : kp * (error) + kd * (error_rate)
  case CONTROL_MODE_MIT:
    ControlImpl_MIT(motor);
    break;
  default:
    /* 未知控制模式：关闭 PWM 输出，防止悬空状态导致硬件损坏。
     * 正常运行时不应进入此分支；若出现，说明 Control_Mode 被意外写入无效值。 */
    if (motor->components.hal && motor->components.hal->pwm) {
      motor->components.hal->pwm->brake();
    }
    ERROR_REPORT(ERROR_MOTOR_ENCODER_LOSS, "Unknown control mode — PWM braked");
    break;
  }
  //
  // open loopmodeFOCinner loop
  if (motor->state.Control_Mode <= CONTROL_MODE_OPEN) {
    return;
  }
  // runningouter loop(speed/velocity/position)
  if (Control_ShouldRunOuterLoops(motor)) {
    Control_OuterLoopsUpdate(motor, &s_ctx);
  }
  // runninginner loop(currentFOC)
  Control_InnerCurrentLoop(motor, &s_ctx);
}
/**
 * @brief set PID
 *
 * @param motor motor
 * @param current_max_out currentoutput (voltage)
 * @param current_max_iout currentintegral
 * @param vel_max_out speed/velocityoutput (current)
 * @param vel_max_iout speed/velocityintegral
 * @param pos_limit positionoutput (speed/velocity)
 */
void SetPIDLimit(MOTOR_DATA *motor, float current_max_out,
                 float current_max_iout, float vel_max_out, float vel_max_iout,
                 float pos_limit) {
  // DaxisQaxisphasevoltage
  motor->IdPID.max_out = current_max_out;
  motor->IdPID.max_iout = current_max_iout;
  motor->IqPID.max_out = current_max_out;
  motor->IqPID.max_iout = current_max_iout;
  motor->VelPID.max_out = vel_max_out;
  motor->VelPID.max_iout = vel_max_iout;
  motor->PosPID.max_out = pos_limit;
  motor->PosPID.max_iout = pos_limit;
}
/**
 * @brief updatecurrentparam (gain)
 */
#define CURRENT_AUTO_CALIBRATION 1 // current
void CurrentLoop_UpdateGain(MOTOR_DATA *motor) {
  // 1. calcgain
#if CURRENT_AUTO_CALIBRATION
  // calc (BW ≈ vel_limit * pole_pairs * 2pi)
  // : calc，actual
  float bandwidth =
      motor->Controller.vel_limit * motor->parameters.pole_pairs * M_2PI;
  motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * bandwidth;
  motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * bandwidth;
#else
  //
  motor->Controller.current_ctrl_p_gain =
      motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth * 1.0f;
  motor->Controller.current_ctrl_i_gain =
      motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth;
#endif
  // 2. gain
  motor->IdPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IdPID.Ki = motor->Controller.current_ctrl_i_gain;
  motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;
  // 3.  (0output)
  // D/Qaxisvoltage = voltage_limit
  float v_limit = motor->Controller.voltage_limit;
  motor->IdPID.max_out = v_limit;
  motor->IdPID.max_iout = v_limit;
  motor->IqPID.max_out = v_limit;
  motor->IqPID.max_iout = v_limit;
  // speed/velocity/positionupdatemode
  motor->VelPID.max_out = motor->Controller.current_limit;
  motor->VelPID.max_iout = motor->Controller.current_limit;
  motor->PosPID.max_out = motor->Controller.vel_limit;
  motor->PosPID.max_iout = motor->Controller.vel_limit;
}
