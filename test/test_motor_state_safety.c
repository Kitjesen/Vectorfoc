// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "current_calib.h"
#include "flux_calib.h"
#include "motor.h"
#include "rsls_calib.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

StateMachine g_ds402_state_machine;
uint8_t g_can_id = 1u;

static unsigned s_enable_count;
static unsigned s_disable_count;
static unsigned s_brake_count;

int MHAL_PWM_Enable(void) {
  s_enable_count++;
  return 0;
}
int MHAL_PWM_Disable(void) {
  s_disable_count++;
  return 0;
}
int MHAL_PWM_Brake(void) {
  s_brake_count++;
  return 0;
}
#undef HAL_GetSystemTick
uint32_t HAL_GetSystemTick(void) { return 0u; }

void PID_clear(PidTypeDef *pid) { memset(pid, 0, sizeof(*pid)); }
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state) {
  memset(state, 0, sizeof(*state));
}
bool MotorControl_Run(MOTOR_DATA *motor) {
  (void)motor;
  return true;
}
void Param_ScheduleSave(void) {}
void RGB_DisplayColorById(uint8_t color_id) { (void)color_id; }
void Safety_Update_Slow(MOTOR_DATA *motor, StateMachine *fsm) {
  (void)motor;
  (void)fsm;
}
bool Safety_HasActiveFault(void) { return false; }
uint32_t Safety_GetActiveFaultBits(void) { return 0u; }
CalibResult CurrentCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx) {
  (void)motor;
  (void)ctx;
  return CALIB_IN_PROGRESS;
}
CalibResult RSLSCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx,
                             float control_period) {
  (void)motor;
  (void)ctx;
  (void)control_period;
  return CALIB_IN_PROGRESS;
}
CalibResult FluxCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx) {
  (void)motor;
  (void)ctx;
  return CALIB_IN_PROGRESS;
}
void Init_Motor_Calib(MOTOR_DATA *motor) { (void)motor; }

int main(void) {
  MOTOR_DATA motor;
  memset(&motor, 0, sizeof(motor));
  StateMachine_Init(&g_ds402_state_machine);

  g_ds402_state_machine.current_state = STATE_FAULT_REACTION_ACTIVE;
  MotorStateTask(&motor);

  assert(motor.state.State_Mode == STATE_MODE_GUARD);
  assert(s_disable_count > 0u);
  assert(s_brake_count == 0u);
  assert(s_enable_count == 0u);

  puts("Motor guard output test: PASS");
  return 0;
}
