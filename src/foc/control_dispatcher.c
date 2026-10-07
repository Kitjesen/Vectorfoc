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

#include "control_dispatcher.h"
#include "pwm_interface.h"
#include "control_context.h"
#include "cogging_compensation.h"
#include "runtime_compensation.h"
#include "safety_manager.h"
#include "algorithm/torque_feedforward.h"
#include "algorithm/foc_current_loop.h"
#include "motor_configuration.h"
#include "error_manager.h"
#include "error_types.h"
#include "algorithm/foc_transforms.h"
#include "algorithm/command_limiter.h"
#include <math.h>

static void Control_SetPidLimits(MOTOR_DATA *motor, float voltage, float current,
                                 float velocity);
static bool Control_RunTorqueMode(MOTOR_DATA *motor);
static void Control_UpdatePositionRamp(MOTOR_DATA *motor, MotorControlCtx *ctx);
static bool Control_RunMitMode(MOTOR_DATA *motor);
static void Control_RunVfMode(MOTOR_DATA *motor);
static void Control_RunOuterLoop(MOTOR_DATA *motor, bool update_position,
                                 bool update_velocity);
static void Control_ScheduleOuterLoops(MOTOR_DATA *motor, bool run_outer);
static void Control_RefreshAlgorithmConfig(MOTOR_DATA *motor);
static void Control_ApplyCurrentLoopOutput(MOTOR_DATA *motor);

// Single-motor control state and command ramps.
static MotorControlCtx s_ctx;
static bool s_limiters_initialized = false;

static void Control_ResetCommandRamps(MOTOR_DATA *motor) {
  float torque = 0.0f;
  if (isfinite(motor->algo_input.Iq_ref) && isfinite(motor->Controller.torque_const))
    torque = motor->algo_input.Iq_ref * motor->Controller.torque_const;
  RateLimiter_Reset(&s_ctx.velocity_limiter, motor->feedback.velocity);
  RateLimiter_Reset(&s_ctx.torque_limiter, torque);
  s_ctx.limited_velocity = motor->feedback.velocity;
  s_ctx.limited_torque = torque;
}

void Control_Initialize(MOTOR_DATA *motor) {
  if (motor == NULL || s_limiters_initialized) return;
  RateLimiter_Init(&s_ctx.velocity_limiter,
      motor->Controller.vel_limit * VELOCITY_ACCEL_MULTIPLIER);
  RateLimiter_Init(&s_ctx.torque_limiter, motor->Controller.torque_ramp_rate);
  Control_ResetCommandRamps(motor);
  s_ctx.last_mode = motor->state.Control_Mode;
  FOC_Algorithm_InitState(&motor->algo_state);
  s_limiters_initialized = true;
}

bool Control_RunCurrentCycle(MOTOR_DATA *motor) {
  if (motor == NULL) return false;
  if (motor->state.Control_Mode != s_ctx.last_mode) {
    Feedforward_Reset();
    FieldWeakening_Reset();
    Control_ResetCommandRamps(motor);
    PID_clear(&motor->VelPID);
    PID_clear(&motor->PosPID);
    LADRC_Reset(&motor->ladrc_state);
    s_ctx.velocity_loop_tick_count = s_ctx.position_loop_tick_count = 0;
    motor->vel_filter_initialized = false;
    motor->Controller.torque_setpoint = 0.0f;
    if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY_RAMP ||
        motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP) {
      motor->Controller.vel_setpoint = motor->feedback.velocity;
      motor->Controller.pos_setpoint = motor->feedback.position;
    }
    s_ctx.last_mode = motor->state.Control_Mode;
  }

  /* Stage 1: advance command ramps at the 20 kHz current-loop rate. */
  RateLimiter_SetMaxRate(&s_ctx.velocity_limiter,
      motor->Controller.vel_limit * VELOCITY_ACCEL_MULTIPLIER);
  RateLimiter_SetMaxRate(&s_ctx.torque_limiter, motor->Controller.torque_ramp_rate);
  s_ctx.limited_velocity = RateLimiter_Apply(&s_ctx.velocity_limiter,
      motor->Controller.input_velocity, COMMAND_RAMP_PERIOD_S);
  s_ctx.limited_torque = RateLimiter_Apply(&s_ctx.torque_limiter,
      motor->Controller.input_torque, COMMAND_RAMP_PERIOD_S);

  /* Stage 2: prepare references for the active control mode.
   * Open-loop voltage modes write PWM directly and end this cycle here. */
  bool run_outer = false;
  switch (motor->state.Control_Mode) {
  case CONTROL_MODE_OPEN: {
    float Ts = CURRENT_SAMPLE_PERIOD_S;
    motor->algo_input.theta_elec = Math_WrapAnglePositive(
        motor->algo_input.theta_elec + 40 * Ts);
    Control_InjectVoltage(motor, 0.0f, OPEN_MODE_FIXED_VOLTAGE,
                          motor->algo_input.theta_elec);
    return true;
  }
  case CONTROL_MODE_VF:
    Control_RunVfMode(motor);
    return true;
  case CONTROL_MODE_TORQUE:
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, 0.0f, 0.0f);
    if (!Control_RunTorqueMode(motor)) return false;
    break;
  case CONTROL_MODE_VELOCITY:
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, 0.0f);
    motor->algo_input.theta_elec = motor->feedback.phase_angle;
    run_outer = true;
    break;
  case CONTROL_MODE_POSITION:
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT,
                         POS_PID_MAX_OUT);
    motor->algo_input.theta_elec = motor->feedback.phase_angle;
    run_outer = true;
    break;
  case CONTROL_MODE_VELOCITY_RAMP: {
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT,
                         POS_PID_MAX_OUT);
    motor->algo_input.theta_elec = motor->feedback.phase_angle;
    float max_step_size =
        fabsf(COMMAND_RAMP_PERIOD_S * motor->Controller.vel_ramp_rate);
    float full_step =
        motor->Controller.input_velocity - motor->Controller.vel_setpoint;
    float step = CLAMP(full_step, -max_step_size, max_step_size);
    motor->Controller.vel_setpoint += step;
    motor->Controller.torque_setpoint =
        Control_InertiaTorque(motor->Controller.inertia, step / COMMAND_RAMP_PERIOD_S);
    run_outer = true;
    break;
  }
  case CONTROL_MODE_POSITION_RAMP:
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT,
                         POS_PID_MAX_OUT);
    Control_UpdatePositionRamp(motor, &s_ctx);
    run_outer = true;
    break;
  case CONTROL_MODE_MIT:
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, 0.0f, 0.0f);
    if (!Control_RunMitMode(motor)) return false;
    break;
  case CONTROL_MODE_IF: {
    Control_SetPidLimits(motor, CURRENT_PID_MAX_OUT, 0.0f, 0.0f);
    float Ts = CURRENT_SAMPLE_PERIOD_S;
    float vel_mech = motor->Controller.input_velocity; // [turn/s]
    float omega_elec = vel_mech * (float)motor->parameters.pole_pairs * M_2PI;
    motor->algo_input.theta_elec = Math_WrapAnglePositive(
        motor->algo_input.theta_elec + omega_elec * Ts);
    float iq = (fabsf(motor->Controller.input_torque) > 0.01f)
                   ? motor->Controller.input_torque
                   : IF_DEFAULT_CURRENT;
    motor->algo_input.Iq_ref = iq;
    motor->algo_input.Id_ref = 0.0f;
    break;
  }
  default:
    motor->algo_input.Id_ref = motor->algo_input.Iq_ref = 0.0f;
    motor->algo_input.enabled = false;
    FOC_Algorithm_ResetState(&motor->algo_state);
    MHAL_PWM_Disable();
    Safety_TriggerFault(FAULT_CONTROL_INVALID, motor, &g_ds402_state_machine);
    return false;
  }

  /* Stage 3: run the independently scheduled position and velocity loops. */
  Control_ScheduleOuterLoops(motor, run_outer);

  /* Stage 4: refresh current-loop inputs, execute FOC, then update PWM. */
  Control_RefreshAlgorithmConfig(motor);
  motor->algo_input.omega_elec =
      motor->feedback.velocity * motor->parameters.pole_pairs * M_2PI;
  motor->algo_input.enabled = true;
  FOC_AlgorithmInput_t effective_input = motor->algo_input;
  effective_input.Iq_ref += CoggingComp_GetCurrent(motor);

  FieldWeakening_Config_t fw_cfg = {
      .max_weakening_current = motor->advanced.fw_max_current,
      .start_velocity = motor->advanced.fw_start_velocity,
  };
  effective_input.Id_ref += FieldWeakening_Calculate(motor, &fw_cfg, FOC_LOOP_PERIOD_S);
  FOC_Algorithm_CurrentLoop(&effective_input, &motor->algo_config,
                            &motor->algo_state, &motor->algo_output);
  Control_ApplyCurrentLoopOutput(motor);
  return true;
}

static void Control_ScheduleOuterLoops(MOTOR_DATA *motor, bool run_outer) {
  if (!run_outer) {
    s_ctx.velocity_loop_tick_count = 0;
    s_ctx.position_loop_tick_count = 0;
    return;
  }

  bool position_mode = motor->state.Control_Mode == CONTROL_MODE_POSITION ||
                       motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP;
  bool velocity_loop_due =
      ++s_ctx.velocity_loop_tick_count >= VELOCITY_LOOP_DECIMATION;
  if (velocity_loop_due) {
    s_ctx.velocity_loop_tick_count = 0;
  }

  bool position_loop_due = false;
  if (position_mode) {
    position_loop_due =
        ++s_ctx.position_loop_tick_count >= POSITION_LOOP_DECIMATION;
    if (position_loop_due) {
      s_ctx.position_loop_tick_count = 0;
    }
  } else {
    s_ctx.position_loop_tick_count = 0;
  }

  if (velocity_loop_due || position_loop_due) {
    Control_RunOuterLoop(motor, position_loop_due, velocity_loop_due);
  }
}

static void Control_RefreshAlgorithmConfig(MOTOR_DATA *motor) {
  if (!motor->params_updated) {
    return;
  }

  motor->algo_config.Rs = motor->parameters.Rs;
  motor->algo_config.Ls = motor->parameters.Ls;
  motor->algo_config.flux = motor->parameters.flux;
  motor->algo_config.pole_pairs = motor->parameters.pole_pairs;
  motor->algo_config.Kp_current_d = motor->IdPID.Kp;
  motor->algo_config.Ki_current_d = motor->IdPID.Ki;
  motor->algo_config.Kp_current_q = motor->IqPID.Kp;
  motor->algo_config.Ki_current_q = motor->IqPID.Ki;
  motor->algo_config.Ts_current = CURRENT_SAMPLE_PERIOD_S;
  motor->algo_config.voltage_limit = motor->Controller.voltage_limit;
  motor->algo_config.current_limit = motor->Controller.current_limit;
  motor->algo_config.enable_decoupling = true;
  motor->params_updated = false;
}

static void Control_ApplyCurrentLoopOutput(MOTOR_DATA *motor) {
  float Ta = motor->algo_output.Ta;
  float Tb = motor->algo_output.Tb;
  float Tc = motor->algo_output.Tc;
#ifdef DEADTIME_COMP
  float deadtime_duty = (float)DEADTIME_COMP / (float)PWM_ARR;
  float Ia = motor->algo_input.Ia;
  float Ib = motor->algo_input.Ib;
  float Ic = motor->algo_input.Ic;

  /* Interpolate through the current zero-crossing to avoid a duty step. */
  float i_thresh = motor->algo_config.deadtime_i_thresh;
  if (i_thresh <= 0.0f) {
    i_thresh = 0.2f;
  }
  float s_a = fabsf(Ia) < i_thresh ? Ia / i_thresh
                                   : (Ia > 0.0f ? 1.0f : -1.0f);
  float s_b = fabsf(Ib) < i_thresh ? Ib / i_thresh
                                   : (Ib > 0.0f ? 1.0f : -1.0f);
  float s_c = fabsf(Ic) < i_thresh ? Ic / i_thresh
                                   : (Ic > 0.0f ? 1.0f : -1.0f);

  float Vbus = motor->algo_input.Vbus;
  float Vdiode = motor->algo_config.deadtime_Vdiode;
  if (Vdiode <= 0.0f) {
    Vdiode = 0.7f;
  }
  float Vcomp =
      Vbus > 1.0f ? deadtime_duty + Vdiode / Vbus : deadtime_duty;

  Ta += Vcomp * s_a;
  Tb += Vcomp * s_b;
  Tc += Vcomp * s_c;
  Ta = CLAMP(Ta, 0.0f, 1.0f);
  Tb = CLAMP(Tb, 0.0f, 1.0f);
  Tc = CLAMP(Tc, 0.0f, 1.0f);
#endif
  MHAL_PWM_SetDuty(Ta, Tb, Tc);
}

/* Adapt the runtime object once at the boundary; the outer loop itself only
 * receives control values and algorithm state handles. */
static void Control_RunOuterLoop(MOTOR_DATA *motor, bool update_position,
                                 bool update_velocity) {
  OuterLoopInput input = {
      .mode = motor->state.Control_Mode,
      .feedback_position = motor->feedback.position,
      .feedback_velocity = motor->feedback.velocity,
      .input_position = motor->Controller.input_position,
      .input_velocity = motor->state.Control_Mode == CONTROL_MODE_VELOCITY
                            ? s_ctx.limited_velocity : motor->Controller.input_velocity,
      .vel_limit = motor->Controller.vel_limit,
      .pos_setpoint = motor->Controller.pos_setpoint,
      .vel_setpoint = motor->Controller.vel_setpoint,
      .torque_setpoint = motor->Controller.torque_setpoint,
      .torque_const = motor->Controller.torque_const,
      .feedforward_current = Feedforward_GetCurrent(motor),
      .position_pid = &motor->PosPID,
      .velocity_pid = &motor->VelPID,
      .ladrc_config = &motor->ladrc_config,
      .ladrc_state = &motor->ladrc_state,
      .ladrc_enable = motor->ladrc_enable,
      .params_updated = motor->params_updated,
      .vel_feedback_filtered = motor->vel_feedback_filtered,
      .vel_filter_initialized = motor->vel_filter_initialized,
  };
  OuterLoopOutput output = {
      .iq_ref = motor->algo_input.Iq_ref,
      .id_ref = motor->algo_input.Id_ref,
      .vel_feedback_filtered = motor->vel_feedback_filtered,
      .vel_filter_initialized = motor->vel_filter_initialized,
  };

  if (update_position) {
    Control_UpdatePositionLoop(&input, &s_ctx);
  }
  if (update_velocity) {
    Control_UpdateVelocityLoop(&input, &output, &s_ctx);
    motor->algo_input.Iq_ref = output.iq_ref;
    motor->algo_input.Id_ref = output.id_ref;
    motor->vel_feedback_filtered = output.vel_feedback_filtered;
    motor->vel_filter_initialized = output.vel_filter_initialized;
  }
}

/* PID outputs and integrators share the limits selected above. */
static void Control_SetPidLimits(MOTOR_DATA *motor, float voltage, float current,
                                 float velocity) {
  motor->IdPID.max_out = motor->IdPID.max_iout = voltage;
  motor->IqPID.max_out = motor->IqPID.max_iout = voltage;
  motor->VelPID.max_out = motor->VelPID.max_iout = current;
  motor->PosPID.max_out = motor->PosPID.max_iout = velocity;
}

void Control_ApplyConfiguredCurrentGains(MOTOR_DATA *motor) {
  if (motor == NULL) return;
  motor->IdPID.Kp = motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IdPID.Ki = motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;
  Control_SetPidLimits(motor, motor->Controller.voltage_limit,
      motor->Controller.current_limit, motor->Controller.vel_limit);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->IqPID);
  FOC_Algorithm_ResetState(&motor->algo_state);
  motor->params_updated = true;
}
void Control_UpdateCurrentGains(MOTOR_DATA *motor) {
  if (motor == NULL) return;
  float bandwidth = (float)motor->Controller.current_ctrl_bandwidth;
  if (bandwidth <= 0.0f)
    bandwidth = motor->Controller.vel_limit * motor->parameters.pole_pairs * M_2PI;
  motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * bandwidth;
  motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * bandwidth;
  Control_ApplyConfiguredCurrentGains(motor);
}

void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle) {
  motor->algo_output.Vd = Vd;
  motor->algo_output.Vq = Vq;
  motor->algo_input.theta_elec = angle;
  // Voltage injection bypasses current PI: dq -> alpha-beta -> SVPWM -> duty.
  Park_Inverse(Vd, Vq, angle, &motor->algo_output.Valpha,
               &motor->algo_output.Vbeta);
  float vbus = motor->algo_input.Vbus;
  if (vbus < VBUS_MIN_VALID_V) {
    vbus = DEFAULT_VBUS_VOLTAGE_V;
  }
  SVPWM_Modulate(motor->algo_output.Valpha, motor->algo_output.Vbeta, vbus,
                 &motor->algo_output.Ta, &motor->algo_output.Tb,
                 &motor->algo_output.Tc);
  MHAL_PWM_SetDuty(motor->algo_output.Ta, motor->algo_output.Tb,
                   motor->algo_output.Tc);
  if (StateMachine_GetState(&g_ds402_state_machine) == STATE_CALIBRATING)
    (void)StateMachine_SetCalibrationPower(&g_ds402_state_machine, true);
}

#ifndef TORQUE_ADJUST
#define TORQUE_ADJUST 0
#endif
#ifndef TORQUE_AND_CURRENT
#define TORQUE_AND_CURRENT 0
#endif

static bool Control_RejectInvalidTorque(MOTOR_DATA *motor) {
  motor->algo_input.Iq_ref = motor->algo_input.Id_ref = 0.0f;
  motor->algo_input.enabled = false;
  MHAL_PWM_Disable();
  Safety_TriggerFault(FAULT_CONTROL_INVALID, motor, &g_ds402_state_machine);
  return false;
}
static bool Control_RunTorqueMode(MOTOR_DATA *motor) {
  motor->algo_input.theta_elec = motor->feedback.phase_angle;
#if TORQUE_AND_CURRENT
#if TORQUE_ADJUST
  motor->algo_input.Iq_ref = 0.0f;
  motor->algo_input.Id_ref = 0.5f;
#else
  motor->Controller.input_current = CLAMP(motor->Controller.input_current,
      -motor->Controller.current_limit, motor->Controller.current_limit);
  motor->algo_input.Iq_ref = motor->Controller.input_current;
  motor->algo_input.Id_ref = 0.0f;
#endif
#else
  float torque = CLAMP(s_ctx.limited_torque, -motor->Controller.torque_limit,
                       motor->Controller.torque_limit);
  float current = 0.0f;
  if (!Control_TorqueToCurrent(torque, motor->Controller.torque_const, &current))
    return Control_RejectInvalidTorque(motor);
  motor->Controller.input_current = CLAMP(current,
      -motor->Controller.current_limit, motor->Controller.current_limit);
  motor->algo_input.Iq_ref = motor->Controller.input_current;
  motor->algo_input.Id_ref = 0.0f;
#endif
  return true;
}

static void Control_UpdatePositionRamp(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  motor->algo_input.theta_elec = motor->feedback.phase_angle;
  // 1) Trajectory Planning (When new target arrives)
  if (motor->Controller.input_updated) {
    TRAJ_plan(&ctx->traj, motor->Controller.input_position, // Target Position
              motor->feedback.position,                    // Current Position
              motor->feedback.velocity,                    // Current Velocity
              motor->Controller.traj_vel,                  // Max Velocity
              motor->Controller.traj_accel,                // Max Acceleration
              motor->Controller.traj_decel);               // Max Deceleration
    ctx->traj.t = 0.0f;
    ctx->traj.trajectory_done = false;
    motor->Controller.input_updated = false;
  }
  // Avoid updating uninitialized trajectory
  if (ctx->traj.trajectory_done) {
    return;
  }
  if (ctx->traj.t > ctx->traj.Tf_) {
    ctx->traj.trajectory_done = true;
    motor->Controller.pos_setpoint = motor->Controller.input_position;
    motor->Controller.vel_setpoint = 0.0f;
    motor->Controller.torque_setpoint = 0.0f;
  } else {
    TRAJ_eval(&ctx->traj, ctx->traj.t);
    motor->Controller.pos_setpoint = ctx->traj.Y;
    motor->Controller.vel_setpoint = ctx->traj.Yd;
    motor->Controller.torque_setpoint =
        Control_InertiaTorque(motor->Controller.inertia, ctx->traj.Ydd);
    if (fabsf(motor->Controller.pos_setpoint - motor->feedback.position) <
        MIT_POSITION_ERROR_TOLERANCE) {
      ctx->traj.t += COMMAND_RAMP_PERIOD_S;
    }
  }
}

static bool Control_RunMitMode(MOTOR_DATA *motor) {
  motor->algo_input.theta_elec = motor->feedback.phase_angle;
  // ========== Parameter Validity Check ==========
  if (motor->Controller.mit_kp < 0.0f || motor->Controller.mit_kd < 0.0f) {
    motor->algo_input.Iq_ref = 0.0f;
    motor->algo_input.Id_ref = 0.0f;
    return true;
  }
  // ========== Unit Conversion: turn -> rad ==========
  float pos_actual_rad = motor->feedback.position * M_2PI;
  float vel_actual_rad = motor->feedback.velocity * M_2PI;
  // ========== MIT Impedance Control: τ = Kp*Δθ + Kd*Δω ==========
  float pos_error = motor->Controller.mit_pos_des - pos_actual_rad;
  float vel_error = motor->Controller.mit_vel_des - vel_actual_rad;
  // ========== Stability Check: Protect if error is too large ==========
  if (fabsf(pos_error) > MIT_POSITION_STABILITY_THRESH ||
      fabsf(vel_error) > MIT_VELOCITY_STABILITY_THRESH) {
    motor->algo_input.Iq_ref *= MIT_MODE_DECAY_FACTOR;
    motor->algo_input.Id_ref = 0.0f;
    return true;
  }
  // Calculate Impedance Torque
  float impedance_torque = motor->Controller.mit_kp * pos_error +
                           motor->Controller.mit_kd * vel_error;
  // ========== Torque Limiting ==========
  float desired_torque =
      CLAMP(impedance_torque + s_ctx.limited_torque,
            -motor->Controller.torque_limit, motor->Controller.torque_limit);
  // ========== Convert to Current and Limit ==========
  float desired_current = 0.0f;
  if (!Control_TorqueToCurrent(desired_torque, motor->Controller.torque_const,
                               &desired_current))
    return Control_RejectInvalidTorque(motor);
  motor->Controller.input_current =
      CLAMP(desired_current, -motor->Controller.current_limit,
            motor->Controller.current_limit);
  motor->algo_input.Iq_ref = motor->Controller.input_current;
  motor->algo_input.Id_ref = 0.0f;
  return true;
}

static void Control_RunVfMode(MOTOR_DATA *motor) {
  float Ts = CURRENT_SAMPLE_PERIOD_S;
  float vel_mech  = motor->Controller.input_velocity; // [turn/s]
  float omega_elec = vel_mech * (float)motor->parameters.pole_pairs * M_2PI;
  // Integrate forced electrical angle
  motor->algo_input.theta_elec = Math_WrapAnglePositive(
      motor->algo_input.theta_elec + omega_elec * Ts);
  // Vq proportional to speed with low-speed boost; sign tracks direction
  float vq = VF_BOOST_VOLTAGE +
             VF_BASE_VOLTAGE * fabsf(vel_mech) / VF_BASE_VELOCITY;
  vq = CLAMP(vq, 0.0f, motor->Controller.voltage_limit);
  if (vel_mech < 0.0f) vq = -vq;
  Control_InjectVoltage(motor, 0.0f, vq, motor->algo_input.theta_elec);
}
