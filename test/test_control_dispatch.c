// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "command_limiter.h"
#include "context.h"
#include "control.h"
#include "impl.h"
#include "inner.h"
#include "outer.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_disable_count;
static int s_inner_count;
static int s_feedforward_reset_count;
static int s_field_weakening_reset_count;
static int s_fault_count;

StateMachine g_ds402_state_machine;

void ControlCommandLimiter_Init(MotorControlCtx *ctx, const MOTOR_DATA *motor) {
  (void)ctx;
  (void)motor;
}
void ControlCommandLimiter_Reset(MotorControlCtx *ctx,
                                 const MOTOR_DATA *motor) {
  (void)ctx;
  (void)motor;
}
void ControlCommandLimiter_Update(MotorControlCtx *ctx, const MOTOR_DATA *motor,
                                  float dt) {
  (void)ctx;
  (void)motor;
  (void)dt;
}
void Feedforward_Reset(void) { s_feedforward_reset_count++; }
void FieldWeakening_Reset(void) { s_field_weakening_reset_count++; }
void PID_clear(PidTypeDef *pid) {
  uint8_t mode = pid->mode;
  float kp = pid->Kp;
  float ki = pid->Ki;
  float kd = pid->Kd;
  float max_out = pid->max_out;
  float max_iout = pid->max_iout;
  float tau = pid->tau;
  memset(pid, 0, sizeof(*pid));
  pid->mode = mode;
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->tau = tau;
}
void LADRC_Reset(LADRC_State_t *state) { memset(state, 0, sizeof(*state)); }
void FOC_Algorithm_InitState(FOC_AlgorithmState_t *state) {
  memset(state, 0, sizeof(*state));
}
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state) {
  memset(state, 0, sizeof(*state));
}
int MHAL_PWM_Disable(void) {
  s_disable_count++;
  return 0;
}
void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)code;
  (void)message;
  (void)file;
  (void)line;
}
void Safety_TriggerFault(uint32_t fault_bits, MOTOR_DATA *motor,
                         StateMachine *fsm) {
  (void)fault_bits;
  (void)motor;
  (void)fsm;
  s_fault_count++;
}

void ControlImpl_SetPidLimits(MOTOR_DATA *motor) { (void)motor; }
void ControlImpl_Open(MOTOR_DATA *motor) { (void)motor; }
bool ControlImpl_Torque(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  (void)motor;
  (void)ctx;
  return true;
}
void ControlImpl_Velocity(MOTOR_DATA *motor) { (void)motor; }
void ControlImpl_Position(MOTOR_DATA *motor) { (void)motor; }
void ControlImpl_VelocityRamp(MOTOR_DATA *motor) { (void)motor; }
void ControlImpl_PositionRamp(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  (void)motor;
  (void)ctx;
}
bool ControlImpl_MIT(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  (void)motor;
  (void)ctx;
  return true;
}
bool Control_ShouldRunOuterLoops(const MOTOR_DATA *motor) {
  (void)motor;
  return false;
}
void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  (void)motor;
  (void)ctx;
}
void Control_InnerCurrentLoop(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  (void)motor;
  (void)ctx;
  s_inner_count++;
}

static bool nearly_equal(float a, float b) { return fabsf(a - b) < 1.0e-5f; }

static bool pid_current_gains_equal(const MOTOR_DATA *motor, float kp,
                                    float ki) {
  return nearly_equal(motor->IdPID.Kp, kp) &&
         nearly_equal(motor->IdPID.Ki, ki) &&
         nearly_equal(motor->IqPID.Kp, kp) && nearly_equal(motor->IqPID.Ki, ki);
}

static int test_current_gain_application(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.Controller.current_ctrl_p_gain = 1.25f;
  motor.Controller.current_ctrl_i_gain = 2.50f;
  motor.Controller.voltage_limit = 18.0f;
  motor.Controller.current_limit = 7.0f;
  motor.Controller.vel_limit = 42.0f;
  motor.IdPID.Iout = 3.0f;
  motor.IqPID.Iout = -2.0f;
  motor.algo_state.integral_d = 4.0f;
  motor.algo_state.integral_q = -5.0f;

  CurrentLoop_ApplyConfiguredGains(&motor);

  if (!pid_current_gains_equal(&motor, 1.25f, 2.50f) ||
      !nearly_equal(motor.IdPID.max_out, 18.0f) ||
      !nearly_equal(motor.IqPID.max_iout, 18.0f) ||
      !nearly_equal(motor.VelPID.max_out, 7.0f) ||
      !nearly_equal(motor.PosPID.max_out, 42.0f) ||
      !nearly_equal(motor.IdPID.Iout, 0.0f) ||
      !nearly_equal(motor.IqPID.Iout, 0.0f) ||
      !nearly_equal(motor.algo_state.integral_d, 0.0f) ||
      !nearly_equal(motor.algo_state.integral_q, 0.0f) ||
      !motor.params_updated) {
    printf("FAIL configured current gains not applied exactly\n");
    return 1;
  }

  return 0;
}

static int test_current_gain_auto_tune(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.parameters.Rs = 0.20f;
  motor.parameters.Ls = 0.0003f;
  motor.parameters.pole_pairs = 7;
  motor.Controller.vel_limit = 50.0f;
  motor.Controller.voltage_limit = 24.0f;
  motor.Controller.current_limit = 8.0f;

  CurrentLoop_UpdateGain(&motor);

  float bandwidth =
      motor.Controller.vel_limit * (float)motor.parameters.pole_pairs * M_2PI;
  float expected_kp = motor.parameters.Ls * bandwidth;
  float expected_ki = motor.parameters.Rs * bandwidth;
  if (!nearly_equal(motor.Controller.current_ctrl_p_gain, expected_kp) ||
      !nearly_equal(motor.Controller.current_ctrl_i_gain, expected_ki) ||
      !pid_current_gains_equal(&motor, expected_kp, expected_ki) ||
      !nearly_equal(motor.IdPID.max_out, 24.0f) ||
      !nearly_equal(motor.VelPID.max_iout, 8.0f) || !motor.params_updated) {
    printf("FAIL current auto tune did not compute/apply expected gains\n");
    return 1;
  }

  return 0;
}

static int test_current_gain_uses_configured_bandwidth(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.parameters.Rs = 0.25f;
  motor.parameters.Ls = 0.0005f;
  motor.parameters.pole_pairs = 7;
  motor.Controller.vel_limit = 10.0f;
  motor.Controller.current_ctrl_bandwidth = 1200;
  motor.Controller.voltage_limit = 24.0f;
  motor.Controller.current_limit = 8.0f;

  CurrentLoop_UpdateGain(&motor);

  float expected_kp = motor.parameters.Ls * 1200.0f;
  float expected_ki = motor.parameters.Rs * 1200.0f;
  if (!nearly_equal(motor.Controller.current_ctrl_p_gain, expected_kp) ||
      !nearly_equal(motor.Controller.current_ctrl_i_gain, expected_ki) ||
      !pid_current_gains_equal(&motor, expected_kp, expected_ki)) {
    printf("FAIL configured current-loop bandwidth was ignored\n");
    return 1;
  }

  return 0;
}

int main(void) {
  if (test_current_gain_application() != 0 ||
      test_current_gain_auto_tune() != 0 ||
      test_current_gain_uses_configured_bandwidth() != 0) {
    return 1;
  }

  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  motor.state.Control_Mode = CONTROL_MODE_OPEN;
  Control_Init(&motor);

  motor.algo_input.Id_ref = 3.0f;
  motor.algo_input.Iq_ref = 4.0f;
  motor.algo_input.enabled = true;
  motor.state.Control_Mode = (CONTROL_MODE)99;
  MotorControl_Run(&motor);

  if (s_disable_count != 1 || s_inner_count != 0 || s_fault_count != 1 ||
      motor.algo_input.Id_ref != 0.0f || motor.algo_input.Iq_ref != 0.0f ||
      motor.algo_input.enabled) {
    printf("FAIL invalid mode did not fail safe\n");
    return 1;
  }
  if (s_feedforward_reset_count != 1 || s_field_weakening_reset_count != 1) {
    printf("FAIL mode change did not reset compensation state\n");
    return 1;
  }

  motor.state.Control_Mode = CONTROL_MODE_TORQUE;
  MotorControl_Run(&motor);
  if (s_feedforward_reset_count != 2 || s_field_weakening_reset_count != 2 ||
      s_inner_count != 1) {
    printf("FAIL re-enable did not reset state and resume control\n");
    return 1;
  }

  printf("Control dispatch and current gain tests passed\n");
  return 0;
}
