// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "foc/svpwm.h"
#include "impl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_disable_count;
static int s_fault_count;

StateMachine g_ds402_state_machine;

int MHAL_PWM_Disable(void) {
  s_disable_count++;
  return 0;
}
int MHAL_PWM_SetDuty(float a, float b, float c) {
  (void)a;
  (void)b;
  (void)c;
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
MotorState StateMachine_GetState(const StateMachine *sm) {
  return sm->current_state;
}
bool StateMachine_SetCalibrationPower(StateMachine *sm, bool enabled) {
  sm->calibration_power_enabled = enabled;
  return true;
}
void SetPIDLimit(MOTOR_DATA *motor, float current_max_out,
                 float current_max_iout, float vel_max_out, float vel_max_iout,
                 float pos_limit) {
  (void)motor;
  (void)current_max_out;
  (void)current_max_iout;
  (void)vel_max_out;
  (void)vel_max_iout;
  (void)pos_limit;
}
void Park_Inverse(float d, float q, float theta, float *alpha, float *beta) {
  (void)d;
  (void)q;
  (void)theta;
  *alpha = 0.0f;
  *beta = 0.0f;
}
int SVPWM_Modulate(float alpha, float beta, float bus, float *a, float *b,
                   float *c) {
  (void)alpha;
  (void)beta;
  (void)bus;
  *a = *b = *c = 0.5f;
  return SVPWM_STATUS_OK;
}
void TRAJ_plan(TrajTypeDef *traj, float xf, float xi, float vi, float vmax,
               float amax, float dmax) {
  (void)traj;
  (void)xf;
  (void)xi;
  (void)vi;
  (void)vmax;
  (void)amax;
  (void)dmax;
}
void TRAJ_eval(TrajTypeDef *traj, float t) {
  (void)traj;
  (void)t;
}

static int assert_failed_safe(const MOTOR_DATA *motor) {
  return s_disable_count == 1 && s_fault_count == 1 &&
         motor->algo_input.Id_ref == 0.0f && motor->algo_input.Iq_ref == 0.0f &&
         !motor->algo_input.enabled;
}

int main(void) {
  MOTOR_DATA motor;
  MotorControlCtx context;
  memset(&motor, 0, sizeof(motor));
  memset(&context, 0, sizeof(context));
  motor.algo_input.Id_ref = 2.0f;
  motor.algo_input.Iq_ref = 3.0f;
  motor.algo_input.enabled = true;
  motor.Controller.torque_limit = 10.0f;
  motor.Controller.current_limit = 20.0f;
  context.limited_torque = 1.0f;

  motor.Controller.torque_const = 0.0f;
  if (ControlImpl_Torque(&motor, &context) || !assert_failed_safe(&motor)) {
    printf("FAIL zero torque constant was not rejected safely\n");
    return 1;
  }

  s_disable_count = 0;
  s_fault_count = 0;
  motor.algo_input.Id_ref = 2.0f;
  motor.algo_input.Iq_ref = 3.0f;
  motor.algo_input.enabled = true;
  motor.Controller.torque_const = NAN;
  motor.Controller.mit_kp = 1.0f;
  motor.Controller.mit_kd = 1.0f;
  if (ControlImpl_MIT(&motor, &context) || !assert_failed_safe(&motor)) {
    printf("FAIL NaN torque constant was not rejected safely\n");
    return 1;
  }

  printf("Control torque conversion fail-safe tests passed\n");
  return 0;
}
