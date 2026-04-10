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

/**
 * @file fsm.c
 * @brief DS402 state
 * @note :
 *   1. prev_controlword update ()
 *   2. EnterFault  FAULT_REACTION_ACTIVE (DS402 )
 *   3. RequestState  (AutoAdvance )
 *   4. ClearFault  (ISR safety)
 *   5. HandleStateEntry state
 *   6. fault ()
 *   7. calibration Shutdown
 */
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_pwm.h"
#include <stddef.h>
#include <string.h>
/* ==========  ========== */
typedef struct {
  MotorState state;
  uint16_t status_mask;
  uint16_t status_value;
} StatusDefinition;
typedef struct {
  uint16_t control_mask;
  uint16_t control_value;
  MotorState from_state;
  MotorState to_state;
} StateTransition;
/* ========== state ========== */
static const StatusDefinition status_table[] = {
    {STATE_NOT_READY_TO_SWITCH_ON, 0x004F, 0x0000},
    {STATE_SWITCH_ON_DISABLED, 0x004F, 0x0040},
    {STATE_READY_TO_SWITCH_ON, 0x006F, 0x0021},
    {STATE_SWITCHED_ON, 0x006F, 0x0023},
    {STATE_OPERATION_ENABLED, 0x006F, 0x0027},
    {STATE_QUICK_STOP_ACTIVE, 0x006F, 0x0007},
    {STATE_FAULT_REACTION_ACTIVE, 0x004F, 0x000F},
    {STATE_FAULT, 0x004F, 0x0008},
    {STATE_CALIBRATING, 0x0100, 0x0100},
};
#define STATUS_TABLE_SIZE (sizeof(status_table) / sizeof(StatusDefinition))
/* ========== DS402 state () ========== */
static const StateTransition transition_table[] = {
    /* Fault Reset: Fault -> Switch On Disabled
     * : actual ProcessStateTransitions  */
    {0x0080, 0x0080, STATE_FAULT, STATE_SWITCH_ON_DISABLED},
    /* Not Ready -> Switch On Disabled (Automatic)
     * : actual StateMachine_Update  */
    {0x0000, 0x0000, STATE_NOT_READY_TO_SWITCH_ON, STATE_SWITCH_ON_DISABLED},
    /* Shutdown: -> Ready to Switch On (Transition 2/6/8) */
    {0x0087, 0x0006, STATE_OPERATION_ENABLED, STATE_READY_TO_SWITCH_ON},
    {0x0087, 0x0006, STATE_SWITCHED_ON, STATE_READY_TO_SWITCH_ON},
    {0x0087, 0x0006, STATE_SWITCH_ON_DISABLED, STATE_READY_TO_SWITCH_ON},
    /* Switch On: Ready -> Switched On (Transition 3) */
    {0x0087, 0x0007, STATE_READY_TO_SWITCH_ON, STATE_SWITCHED_ON},
    /* Disable Voltage: -> Switch On Disabled (Transition 7/9/10/12)
     *  Quick Stop */
    {0x0002, 0x0000, STATE_READY_TO_SWITCH_ON, STATE_SWITCH_ON_DISABLED},
    {0x0002, 0x0000, STATE_SWITCHED_ON, STATE_SWITCH_ON_DISABLED},
    {0x0002, 0x0000, STATE_OPERATION_ENABLED, STATE_SWITCH_ON_DISABLED},
    {0x0002, 0x0000, STATE_QUICK_STOP_ACTIVE, STATE_SWITCH_ON_DISABLED},
    /* Disable Operation: Operation Enabled -> Switched On (Transition 5) */
    {0x000F, 0x0007, STATE_OPERATION_ENABLED, STATE_SWITCHED_ON},
    /* Enable Operation: -> Operation Enabled (Transition 4/16) */
    {0x000F, 0x000F, STATE_SWITCHED_ON, STATE_OPERATION_ENABLED},
    {0x000F, 0x000F, STATE_QUICK_STOP_ACTIVE, STATE_OPERATION_ENABLED},
    /* Quick Stop: Operation Enabled -> Quick Stop Active (Transition 11) */
    {0x0004, 0x0000, STATE_OPERATION_ENABLED, STATE_QUICK_STOP_ACTIVE},
    /* --- state (DS402) --- */
    /* calibration: Ready to Switch On -> Calibrating (Bit8 ) */
    {0x0100, 0x0100, STATE_READY_TO_SWITCH_ON, STATE_CALIBRATING},
    /* calibration Shutdown: Calibrating -> Ready to Switch On (safety) */
    {0x0087, 0x0006, STATE_CALIBRATING, STATE_READY_TO_SWITCH_ON},
    /* calibration Disable Voltage: Calibrating -> Switch On Disabled */
    {0x0002, 0x0000, STATE_CALIBRATING, STATE_SWITCH_ON_DISABLED},
};
#define TRANSITION_TABLE_SIZE                                                   \
  (sizeof(transition_table) / sizeof(StateTransition))
/* ==========  ========== */
static void UpdateStatusword(StateMachine *sm) {
  uint16_t status = 0;
  for (size_t i = 0; i < STATUS_TABLE_SIZE; i++) {
    if (status_table[i].state == sm->current_state) {
      status = status_table[i].status_value;
      break;
    }
  }
  sm->statusword.word = status;
  sm->statusword.bits.voltage_enabled = 1;
}
static void HandleStateEntry(StateMachine *sm) {
  switch (sm->current_state) {
  case STATE_OPERATION_ENABLED:
    MHAL_PWM_Enable();
    break;
  case STATE_QUICK_STOP_ACTIVE:
    /* stop: PWM
     *  SWITCH_ON_DISABLED  */
    break;
  case STATE_FAULT_REACTION_ACTIVE:
    /* fault:  PWM safety */
    MHAL_PWM_Disable();
    break;
  case STATE_NOT_READY_TO_SWITCH_ON:
  case STATE_SWITCH_ON_DISABLED:
  case STATE_FAULT:
    MHAL_PWM_Disable();
    break;
  default:
    break;
  }
}
static void ExecuteTransition(StateMachine *sm, MotorState new_state) {
  if (sm->current_state == new_state) {
    return;
  }
  /* check */
  if (sm->pre_check_callback && !sm->pre_check_callback(new_state)) {
    return; /*  */
  }
  sm->current_state = new_state;
  sm->state_entry_time = HAL_GetSystemTick();
  HandleStateEntry(sm);
  UpdateStatusword(sm);
}
static bool IsBitEdgeRising(uint16_t prev, uint16_t curr, uint16_t bit_mask) {
  return ((curr & bit_mask) == bit_mask) && ((prev & bit_mask) == 0);
}
static void ProcessStateTransitions(StateMachine *sm) {
  uint16_t cw = sm->controlword.word;
  uint16_t prev_cw = sm->prev_controlword;
  /* [FIX] update，update
   * update， early return  */
  sm->prev_controlword = cw;
  /* 1. faultreset (: 0->1) */
  if (sm->current_state == STATE_FAULT) {
    if (IsBitEdgeRising(prev_cw, cw, 0x0080)) {
      ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED);
    }
    return;
  }
  /* 2.  () */
  for (size_t i = 0; i < TRANSITION_TABLE_SIZE; i++) {
    const StateTransition *t = &transition_table[i];
    if (sm->current_state != t->from_state) {
      continue;
    }
    if ((cw & t->control_mask) == t->control_value) {
      ExecuteTransition(sm, t->to_state);
      return;
    }
  }
  /* 3. : FAULT_REACTION_ACTIVE -> FAULT (DS402) */
  if (sm->current_state == STATE_FAULT_REACTION_ACTIVE) {
    ExecuteTransition(sm, STATE_FAULT);
  }
}
/**
 * @brief statetargetstatecalc
 * @note  DS402 state:
 *   SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON -> SWITCHED_ON -> OPERATION_ENABLED
 *    Update ，doneactual
 */
static void AutoAdvanceToTarget(StateMachine *sm) {
  if (!sm->auto_advance) {
    return;
  }
  /* targetstate */
  if (sm->target_state == sm->current_state) {
    sm->auto_advance = false;
    return;
  }
  /* faultstate -  */
  if (sm->current_state == STATE_FAULT ||
      sm->current_state == STATE_FAULT_REACTION_ACTIVE) {
    sm->auto_advance = false;
    return;
  }
  MotorState target = sm->target_state;
  switch (sm->current_state) {
  case STATE_NOT_READY_TO_SWITCH_ON:
    /* waitinit SWITCH_ON_DISABLED ( Update ) */
    break;
  case STATE_SWITCH_ON_DISABLED:
    /* : -> READY_TO_SWITCH_ON (Shutdown ) */
    if (target == STATE_READY_TO_SWITCH_ON ||
        target == STATE_SWITCHED_ON ||
        target == STATE_OPERATION_ENABLED ||
        target == STATE_CALIBRATING) {
      sm->controlword.word = 0x0006; /* Shutdown */
    }
    break;
  case STATE_READY_TO_SWITCH_ON:
    if (target == STATE_CALIBRATING) {
      sm->controlword.word = 0x0106; /* calibration (Bit8 + enable_voltage + quick_stop) */
    } else if (target == STATE_SWITCHED_ON ||
               target == STATE_OPERATION_ENABLED) {
      sm->controlword.word = 0x0007; /* Switch On */
    } else if (target == STATE_SWITCH_ON_DISABLED) {
      sm->controlword.word = 0x0000; /* Disable Voltage */
    }
    break;
  case STATE_SWITCHED_ON:
    if (target == STATE_OPERATION_ENABLED) {
      sm->controlword.word = 0x000F; /* Enable Operation */
    } else if (target == STATE_READY_TO_SWITCH_ON ||
               target == STATE_CALIBRATING) {
      sm->controlword.word = 0x0006; /* Shutdown -> READY_TO_SWITCH_ON */
    } else if (target == STATE_SWITCH_ON_DISABLED) {
      sm->controlword.word = 0x0000; /* Disable Voltage */
    }
    break;
  case STATE_OPERATION_ENABLED:
    if (target == STATE_SWITCHED_ON) {
      sm->controlword.word = 0x0007; /* Disable Operation */
    } else if (target == STATE_READY_TO_SWITCH_ON ||
               target == STATE_CALIBRATING) {
      sm->controlword.word = 0x0006; /* Shutdown -> READY_TO_SWITCH_ON */
    } else if (target == STATE_SWITCH_ON_DISABLED) {
      sm->controlword.word = 0x0000; /* Disable Voltage */
    }
    break;
  case STATE_QUICK_STOP_ACTIVE:
    if (target == STATE_OPERATION_ENABLED) {
      sm->controlword.word = 0x000F; /* Re-enable */
    } else {
      sm->controlword.word = 0x0000; /* Disable Voltage */
    }
    break;
  case STATE_CALIBRATING:
    if (target == STATE_SWITCH_ON_DISABLED) {
      sm->controlword.word = 0x0000; /* Disable Voltage */
    } else if (target == STATE_READY_TO_SWITCH_ON) {
      sm->controlword.word = 0x0006; /* Shutdown (calibrationsafety) */
    }
    break;
  default:
    sm->auto_advance = false;
    break;
  }
}
/* ==========  API  ========== */
void StateMachine_Init(StateMachine *sm) {
  memset(sm, 0, sizeof(StateMachine));
  sm->current_state = STATE_NOT_READY_TO_SWITCH_ON;
  sm->target_state = STATE_NOT_READY_TO_SWITCH_ON;
  UpdateStatusword(sm);
}
void StateMachine_Update(StateMachine *sm) {
  /* init */
  if (sm->current_state == STATE_NOT_READY_TO_SWITCH_ON) {
    ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED);
  }
  /* targetstate ( RequestState ) */
  AutoAdvanceToTarget(sm);
  ProcessStateTransitions(sm);
  UpdateStatusword(sm);
}
bool StateMachine_RequestState(StateMachine *sm, MotorState target_state) {
  if ((uint32_t)target_state >= STATE_COUNT) {
    return false;
  }
  sm->target_state = target_state;
  sm->auto_advance = true;
  /* calc */
  AutoAdvanceToTarget(sm);
  return true;
}
void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword) {
  sm->controlword.word = controlword;
  sm->auto_advance = false; /* ，stop */
}
uint16_t StateMachine_GetStatusword(const StateMachine *sm) {
  return sm->statusword.word;
}
MotorState StateMachine_GetState(const StateMachine *sm) {
  return sm->current_state;
}
void StateMachine_EnterFault(StateMachine *sm, uint32_t fault_code) {
  /* fault (per-instance) */
  sm->fault_history[sm->fault_history_index] = fault_code;
  sm->fault_history_index = (sm->fault_history_index + 1) % FAULT_HISTORY_SIZE;
  sm->active_fault_code = fault_code;
  /* stop */
  sm->auto_advance = false;
  /* [FIX] DS402:  FAULT_REACTION_ACTIVE safety，
   *  ProcessStateTransitions  FAULT */
  if (sm->current_state != STATE_FAULT &&
      sm->current_state != STATE_FAULT_REACTION_ACTIVE) {
    ExecuteTransition(sm, STATE_FAULT_REACTION_ACTIVE);
  }
}
bool StateMachine_ClearFault(StateMachine *sm) {
  if (sm->current_state != STATE_FAULT) {
    return false;
  }
  /* [FIX] safetystate，
   *  __disable_irq  Update ，safety */
  sm->active_fault_code = 0;
  ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED);
  return true;
}
void StateMachine_SetPreCheckCallback(StateMachine *sm,
                                      bool (*callback)(MotorState to_state)) {
  sm->pre_check_callback = callback;
}
