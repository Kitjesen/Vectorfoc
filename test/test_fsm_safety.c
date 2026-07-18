// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "fsm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static bool s_pwm_enabled;
static bool s_enable_should_fail;
static unsigned s_enable_count;
static unsigned s_disable_count;
static unsigned s_irq_depth;
static unsigned s_irq_max_depth;

uint32_t HAL_EnterCritical(void) {
  s_irq_depth++;
  if (s_irq_depth > s_irq_max_depth) {
    s_irq_max_depth = s_irq_depth;
  }
  return s_irq_depth;
}

void HAL_ExitCritical(uint32_t state) {
  (void)state;
  assert(s_irq_depth > 0u);
  s_irq_depth--;
}

int MHAL_PWM_Enable(void) {
  if (s_enable_should_fail) {
    return -1;
  }
  s_pwm_enabled = true;
  s_enable_count++;
  return 0;
}

int MHAL_PWM_Disable(void) {
  s_pwm_enabled = false;
  s_disable_count++;
  return 0;
}

int MHAL_PWM_Brake(void) { return 0; }
uint32_t HAL_GetSystemTick(void) { return 0; }

static void advance_to_operation_enabled(StateMachine *sm) {
  assert(StateMachine_RequestState(sm, STATE_OPERATION_ENABLED));
  for (unsigned i = 0; i < 4; ++i) {
    StateMachine_Update(sm);
  }
  assert(StateMachine_GetState(sm) == STATE_OPERATION_ENABLED);
}

static void advance_to_calibrating(StateMachine *sm) {
  assert(StateMachine_RequestState(sm, STATE_CALIBRATING));
  for (unsigned i = 0; i < 5 && StateMachine_GetState(sm) != STATE_CALIBRATING;
       ++i) {
    StateMachine_Update(sm);
  }
  assert(StateMachine_GetState(sm) == STATE_CALIBRATING);
}

static void advance_to_state(StateMachine *sm, MotorState target) {
  assert(StateMachine_RequestState(sm, target));
  for (unsigned i = 0; i < 8 && StateMachine_GetState(sm) != target; ++i) {
    StateMachine_Update(sm);
  }
  assert(StateMachine_GetState(sm) == target);
}

static void simulate_flash_save_without_global_irq_lock(StateMachine *sm) {
  assert(s_irq_depth == 0u);
  assert(StateMachine_BeginMaintenance(sm));
  assert(s_irq_depth == 0u);

  for (unsigned i = 0; i < 16; ++i) {
    assert(s_irq_depth == 0u);
  }

  StateMachine_EndMaintenance(sm);
  assert(s_irq_depth == 0u);
}

int main(void) {
  StateMachine sm;
  StateMachine_Init(&sm);

  assert(!sm.statusword.bits.voltage_enabled);
  advance_to_operation_enabled(&sm);
  assert(!s_pwm_enabled);
  assert(!sm.statusword.bits.voltage_enabled);
  assert(StateMachine_SetOperationPower(&sm, true));
  assert(s_pwm_enabled);
  assert(sm.statusword.bits.voltage_enabled);

  StateMachine_Init(&sm);
  advance_to_operation_enabled(&sm);
  s_pwm_enabled = false;
  s_enable_should_fail = true;
  assert(!StateMachine_SetOperationPower(&sm, true));
  assert(!s_pwm_enabled);
  assert(!sm.operation_power_enabled);
  assert(!sm.calibration_power_enabled);
  assert(!sm.statusword.bits.voltage_enabled);
  assert(StateMachine_GetState(&sm) == STATE_FAULT_REACTION_ACTIVE);
  s_enable_should_fail = false;

  StateMachine_Init(&sm);
  advance_to_operation_enabled(&sm);
  assert(StateMachine_SetOperationPower(&sm, true));
  const unsigned enable_count_before_fault = s_enable_count;
  StateMachine_EnterFault(&sm, 0x1234u);
  assert(StateMachine_GetState(&sm) == STATE_FAULT_REACTION_ACTIVE);
  assert(!s_pwm_enabled);
  assert(!sm.statusword.bits.voltage_enabled);

  StateMachine_Update(&sm);
  assert(StateMachine_GetState(&sm) == STATE_FAULT);
  assert(!s_pwm_enabled);
  assert(!sm.statusword.bits.voltage_enabled);
  assert(s_enable_count == enable_count_before_fault);
  assert(s_disable_count >= 2u);

  StateMachine_Init(&sm);
  advance_to_calibrating(&sm);
  assert(!s_pwm_enabled);
  assert(!sm.statusword.bits.voltage_enabled);

  assert(StateMachine_SetCalibrationPower(&sm, true));
  assert(s_pwm_enabled);
  assert(sm.statusword.bits.voltage_enabled);

  assert(StateMachine_SetCalibrationPower(&sm, false));
  assert(!s_pwm_enabled);
  assert(!sm.statusword.bits.voltage_enabled);

  StateMachine_Init(&sm);
  advance_to_calibrating(&sm);
  s_pwm_enabled = false;
  s_enable_should_fail = true;
  assert(!StateMachine_SetCalibrationPower(&sm, true));
  assert(!s_pwm_enabled);
  assert(!sm.operation_power_enabled);
  assert(!sm.calibration_power_enabled);
  assert(!sm.statusword.bits.voltage_enabled);
  assert(StateMachine_GetState(&sm) == STATE_FAULT_REACTION_ACTIVE);
  s_enable_should_fail = false;

  assert(StateMachine_RequestState(&sm, STATE_SWITCH_ON_DISABLED));
  StateMachine_Update(&sm);
  assert(!s_pwm_enabled);
  assert(!sm.statusword.bits.voltage_enabled);

  StateMachine_Init(&sm);
  StateMachine_Update(&sm);
  assert(StateMachine_GetState(&sm) == STATE_SWITCH_ON_DISABLED);
  assert(StateMachine_BeginMaintenance(&sm));
  assert(!StateMachine_BeginMaintenance(&sm));
  assert(!StateMachine_RequestState(&sm, STATE_OPERATION_ENABLED));
  StateMachine_SetControlword(&sm, 0x000F);
  StateMachine_Update(&sm);
  assert(StateMachine_GetState(&sm) == STATE_SWITCH_ON_DISABLED);
  assert(!StateMachine_RequestState(&sm, STATE_CALIBRATING));
  StateMachine_SetControlword(&sm, 0x0106);
  StateMachine_Update(&sm);
  assert(StateMachine_GetState(&sm) == STATE_READY_TO_SWITCH_ON);
  StateMachine_Update(&sm);
  assert(StateMachine_GetState(&sm) == STATE_READY_TO_SWITCH_ON);
  StateMachine_EndMaintenance(&sm);
  assert(StateMachine_RequestState(&sm, STATE_OPERATION_ENABLED));
  for (unsigned i = 0; i < 4; ++i) {
    StateMachine_Update(&sm);
  }
  assert(StateMachine_GetState(&sm) == STATE_OPERATION_ENABLED);

  StateMachine_Init(&sm);
  advance_to_operation_enabled(&sm);
  assert(!StateMachine_BeginMaintenance(&sm));
  advance_to_state(&sm, STATE_SWITCH_ON_DISABLED);
  assert(StateMachine_GetState(&sm) != STATE_OPERATION_ENABLED);
  assert(StateMachine_BeginMaintenance(&sm));
  StateMachine_EndMaintenance(&sm);

  s_irq_max_depth = 0u;
  simulate_flash_save_without_global_irq_lock(&sm);
  assert(s_irq_depth == 0u);
  assert(s_irq_max_depth == 1u);

  puts("FSM fail-safe output test: PASS");
  return 0;
}
