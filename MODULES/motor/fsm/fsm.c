#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_pwm.h"
#include <stddef.h>
#include <string.h>

/* 内部辅助结构 */

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

/* 状态字定义表 */

static const StatusDefinition status_table[] = {
    {STATE_NOT_READY_TO_SWITCH_ON, 0x004F, 0x0000},
    {STATE_SWITCH_ON_DISABLED, 0x004F, 0x0040},
    {STATE_READY_TO_SWITCH_ON, 0x006F, 0x0021},
    {STATE_SWITCHED_ON, 0x006F, 0x0023},
    {STATE_OPERATION_ENABLED, 0x006F, 0x0027},
    {STATE_QUICK_STOP_ACTIVE, 0x006F, 0x0007},
    {STATE_FAULT_REACTION_ACTIVE, 0x004F, 0x000F},
    {STATE_FAULT, 0x004F, 0x0008},
    {STATE_CALIBRATING, 0x0100, 0x0100}};

#define STATUS_TABLE_SIZE (sizeof(status_table) / sizeof(StatusDefinition))

/* DS402 状态转换表 */

static const StateTransition transition_table[] = {
    // Fault Reset: Fault -> Switch On Disabled
    {0x0080, 0x0080, STATE_FAULT, STATE_SWITCH_ON_DISABLED},

    // Not Ready -> Switch On Disabled (Automatic)
    {0x0000, 0x0000, STATE_NOT_READY_TO_SWITCH_ON, STATE_SWITCH_ON_DISABLED},

    // Shutdown: Operation Enabled / Switched On -> Ready to Switch On
    {0x0087, 0x0006, STATE_OPERATION_ENABLED, STATE_READY_TO_SWITCH_ON},
    {0x0087, 0x0006, STATE_SWITCHED_ON, STATE_READY_TO_SWITCH_ON},
    {0x0087, 0x0006, STATE_SWITCH_ON_DISABLED, STATE_READY_TO_SWITCH_ON},

    // Switch On: Ready -> Switched On
    {0x0087, 0x0007, STATE_READY_TO_SWITCH_ON, STATE_SWITCHED_ON},

    // Disable Voltage: Any Active State -> Switch On Disabled
    {0x0002, 0x0000, STATE_READY_TO_SWITCH_ON, STATE_SWITCH_ON_DISABLED},
    {0x0002, 0x0000, STATE_SWITCHED_ON, STATE_SWITCH_ON_DISABLED},
    {0x0002, 0x0000, STATE_OPERATION_ENABLED, STATE_SWITCH_ON_DISABLED},
    {0x0002, 0x0000, STATE_QUICK_STOP_ACTIVE, STATE_SWITCH_ON_DISABLED},

    // Disable Operation: Operation Enabled -> Switched On
    {0x000F, 0x0007, STATE_OPERATION_ENABLED, STATE_SWITCHED_ON},

    // Enable Operation: Switched On / Quick Stop -> Operation Enabled
    {0x000F, 0x000F, STATE_SWITCHED_ON, STATE_OPERATION_ENABLED},
    {0x000F, 0x000F, STATE_QUICK_STOP_ACTIVE, STATE_OPERATION_ENABLED},

    // Quick Stop: Operation Enabled -> Quick Stop Active
    {0x0004, 0x0000, STATE_OPERATION_ENABLED, STATE_QUICK_STOP_ACTIVE},

    /* --- Extended States --- */
    // Enter Calibration: Ready to Switch On -> Calibrating (Bit 8 set)
    {0x0100, 0x0100, STATE_READY_TO_SWITCH_ON, STATE_CALIBRATING},

    // Exit Calibration: Calibrating -> Switch On Disabled (Disable Voltage)
    {0x0002, 0x0000, STATE_CALIBRATING, STATE_SWITCH_ON_DISABLED},
};

#define TRANSITION_TABLE_SIZE                                                  \
  (sizeof(transition_table) / sizeof(StateTransition))

/* 故障历史记录 */

#define FAULT_HISTORY_SIZE 5
static uint32_t s_fault_history[FAULT_HISTORY_SIZE] = {0};
static uint8_t s_fault_history_index = 0;

/* 内部辅助函数 */

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
  case STATE_SWITCH_ON_DISABLED:
  case STATE_FAULT:
    MHAL_PWM_Disable();
    break;
  default:
    break;
  }
}

static void ExecuteTransition(StateMachine *sm, MotorState new_state) {
  if (sm->current_state != new_state) {
    // Check Guard Condition
    if (sm->pre_check_callback && !sm->pre_check_callback(new_state)) {
      return; // Transition denied
    }

    sm->current_state = new_state;
    sm->state_entry_time = HAL_GetSystemTick();

    HandleStateEntry(sm);
    UpdateStatusword(sm);
  }
}

static bool IsBitEdgeRising(uint16_t prev, uint16_t curr, uint16_t bit_mask) {
  return ((curr & bit_mask) == bit_mask) && ((prev & bit_mask) == 0);
}

static void ProcessStateTransitions(StateMachine *sm) {
  uint16_t cw = sm->controlword.word;
  uint16_t prev_cw = sm->prev_controlword;

  // 1. Handle Fault Reset (Edge Triggered: 0->1)
  if (sm->current_state == STATE_FAULT) {
    if (IsBitEdgeRising(prev_cw, cw, 0x0080)) { // Bit 7: Fault Reset
      ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED);
    }
    return;
  }

  // 2. Standard Transitions
  for (size_t i = 0; i < TRANSITION_TABLE_SIZE; i++) {
    const StateTransition *t = &transition_table[i];

    if (sm->current_state != t->from_state) {
      continue;
    }

    // Mask check
    if ((cw & t->control_mask) == t->control_value) {
      ExecuteTransition(sm, t->to_state);
      return;
    }
  }

  // 3. Auto Transitions
  if (sm->current_state == STATE_FAULT_REACTION_ACTIVE) {
    ExecuteTransition(sm, STATE_FAULT);
  }

  // Update history
  sm->prev_controlword = cw;
}

/* 公共API实现 (按照 fsm.h 声明顺序) */

void StateMachine_Init(StateMachine *sm) {
  memset(sm, 0, sizeof(StateMachine));
  sm->current_state = STATE_NOT_READY_TO_SWITCH_ON;
  sm->target_state = STATE_NOT_READY_TO_SWITCH_ON;
  sm->statusword.bits.switch_on_disabled = 1;
}

void StateMachine_Update(StateMachine *sm) {
  if (sm->current_state == STATE_NOT_READY_TO_SWITCH_ON) {
    ExecuteTransition(sm, STATE_SWITCH_ON_DISABLED);
  }

  ProcessStateTransitions(sm);
  UpdateStatusword(sm);
}

bool StateMachine_RequestState(StateMachine *sm, MotorState target_state) {
  sm->target_state = target_state;

  switch (target_state) {
  case STATE_OPERATION_ENABLED:
    sm->controlword.word = 0x0F;
    break;

  case STATE_SWITCHED_ON:
    sm->controlword.word = 0x07;
    break;

  case STATE_READY_TO_SWITCH_ON:
    sm->controlword.word = 0x06;
    break;

  case STATE_SWITCH_ON_DISABLED:
    sm->controlword.word = 0x00;
    break;

  case STATE_CALIBRATING:
    // Set Bit 8 to trigger calibration transition
    // Also set Bit 1,2 (0x06) to ensure transition from Switch On Disabled ->
    // Ready To Switch On
    sm->controlword.word = 0x0106;
    break;

  default:
    return false;
  }

  return true;
}

void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword) {
  sm->controlword.word = controlword;
}

uint16_t StateMachine_GetStatusword(const StateMachine *sm) {
  return sm->statusword.word;
}

MotorState StateMachine_GetState(const StateMachine *sm) {
  return sm->current_state;
}

void StateMachine_EnterFault(StateMachine *sm, uint32_t fault_code) {
  s_fault_history[s_fault_history_index] = fault_code;
  s_fault_history_index = (s_fault_history_index + 1) % FAULT_HISTORY_SIZE;

  ExecuteTransition(sm, STATE_FAULT);
}

// End of file
bool StateMachine_ClearFault(StateMachine *sm) {
  if (sm->current_state == STATE_FAULT) {
    // Simulate edge
    sm->controlword.bits.fault_reset = 0;
    StateMachine_Update(sm);
    sm->controlword.bits.fault_reset = 1;
    StateMachine_Update(sm);
    sm->controlword.bits.fault_reset = 0;
    return true;
  }
  return false;
}

void StateMachine_SetPreCheckCallback(StateMachine *sm,
                                      bool (*callback)(MotorState to_state)) {
  sm->pre_check_callback = callback;
}
