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
#include "command_limiter.h"
#include "context.h"
#include "feedforward.h"
#include "field_weakening.h"
#include "inner.h"
#include "outer.h"
#include "impl.h"
#include "config.h"
#include "foc/foc_algorithm.h"
#include "trajectory/rate_limiter.h"
#include "error_manager.h"
#include "error_types.h"
#include "hal_pwm.h"
#include "safety_control.h"
// Static Context
static MotorControlCtx s_ctx;
static bool s_limiters_initialized = false;

static void Control_HandleModeTransition(MOTOR_DATA *motor) {
  if (motor->state.Control_Mode == s_ctx.last_mode)
    return;

  Feedforward_Reset();
  FieldWeakening_Reset();
  ControlCommandLimiter_Reset(&s_ctx, motor);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);
  LADRC_Reset(&motor->ladrc_state);
  s_ctx.loop_count = 0;
  motor->vel_filter_initialized = false;
  motor->Controller.torque_setpoint = 0.0f;

  if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY_RAMP ||
      motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP) {
    motor->Controller.vel_setpoint = motor->feedback.velocity;
    motor->Controller.pos_setpoint = motor->feedback.position;
  }

  s_ctx.last_mode = motor->state.Control_Mode;
}

void Control_Init(MOTOR_DATA *motor) {
  if (motor == NULL)
    return;
  if (s_limiters_initialized) {
    return;
  }
  ControlCommandLimiter_Init(&s_ctx, motor);
  s_ctx.last_mode = motor->state.Control_Mode;
  // initFOCstate ( MOTOR_DATA )
  FOC_Algorithm_InitState(&motor->algo_state);
  s_limiters_initialized = true;
}
bool MotorControl_Run(MOTOR_DATA *motor) {
  if (motor == NULL)
    return false;
  Control_HandleModeTransition(motor);
  ControlCommandLimiter_Update(&s_ctx, motor, CURRENT_MEASURE_PERIOD);
  // mode: limit, ,
  ControlImpl_SetPidLimits(motor);
  switch (motor->state.Control_Mode) {
  // open loop
  case CONTROL_MODE_OPEN:
    ControlImpl_Open(motor);
    break;
  //
  case CONTROL_MODE_TORQUE:
    if (!ControlImpl_Torque(motor, &s_ctx))
      return false;
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
    if (!ControlImpl_MIT(motor, &s_ctx))
      return false;
    break;
  default:
    /* 未知控制模式：关闭 PWM 输出，防止悬空状态导致硬件损坏。
     * 正常运行时不应进入此分支；若出现，说明 Control_Mode 被意外写入无效值。 */
    motor->algo_input.Id_ref = 0.0f;
    motor->algo_input.Iq_ref = 0.0f;
    motor->algo_input.enabled = false;
    FOC_Algorithm_ResetState(&motor->algo_state);
    MHAL_PWM_Disable();
    Safety_TriggerFault(FAULT_CONTROL_INVALID, motor,
                        &g_ds402_state_machine);
    return false;
  }
  //
  // open loopmodeFOCinner loop
  if (motor->state.Control_Mode <= CONTROL_MODE_OPEN) {
    return true;
  }
  // runningouter loop(speed/velocity/position)
  if (Control_ShouldRunOuterLoops(motor)) {
    Control_OuterLoopsUpdate(motor, &s_ctx);
  }
  // runninginner loop(currentFOC)
  Control_InnerCurrentLoop(motor, &s_ctx);
  return true;
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

static void CurrentLoop_ApplyLimits(MOTOR_DATA *motor) {
  float v_limit = motor->Controller.voltage_limit;
  motor->IdPID.max_out = v_limit;
  motor->IdPID.max_iout = v_limit;
  motor->IqPID.max_out = v_limit;
  motor->IqPID.max_iout = v_limit;
  motor->VelPID.max_out = motor->Controller.current_limit;
  motor->VelPID.max_iout = motor->Controller.current_limit;
  motor->PosPID.max_out = motor->Controller.vel_limit;
  motor->PosPID.max_iout = motor->Controller.vel_limit;
}

void CurrentLoop_ApplyConfiguredGains(MOTOR_DATA *motor) {
  if (motor == NULL)
    return;

  motor->IdPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IdPID.Ki = motor->Controller.current_ctrl_i_gain;
  motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;
  CurrentLoop_ApplyLimits(motor);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->IqPID);
  FOC_Algorithm_ResetState(&motor->algo_state);
  motor->params_updated = true;
}
/**
 * @brief updatecurrentparam (gain)
 */
#define CURRENT_AUTO_CALIBRATION 1 // current
void CurrentLoop_UpdateGain(MOTOR_DATA *motor) {
  if (motor == NULL)
    return;

  // 1. calcgain
#if CURRENT_AUTO_CALIBRATION
  float bandwidth = (float)motor->Controller.current_ctrl_bandwidth;
  if (bandwidth <= 0.0f) {
    /* Legacy fallback for callers that have not configured bandwidth yet. */
    bandwidth =
        motor->Controller.vel_limit * motor->parameters.pole_pairs * M_2PI;
  }
  motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * bandwidth;
  motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * bandwidth;
#else
  //
  motor->Controller.current_ctrl_p_gain =
      motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth * 1.0f;
  motor->Controller.current_ctrl_i_gain =
      motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth;
#endif
  CurrentLoop_ApplyConfiguredGains(motor);
}
