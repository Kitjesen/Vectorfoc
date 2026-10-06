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

#ifndef FSM_H
#define FSM_H
#include <stdbool.h>
#include <stdint.h>
/* statestate (CANopen DS402) */
typedef enum {
  STATE_NOT_READY_TO_SWITCH_ON = 0, //
  STATE_SWITCH_ON_DISABLED,         //
  STATE_READY_TO_SWITCH_ON,         //
  STATE_SWITCHED_ON,                //
  STATE_OPERATION_ENABLED,          // runningenable
  STATE_QUICK_STOP_ACTIVE,          // stop
  STATE_FAULT_REACTION_ACTIVE,      // fault
  STATE_FAULT,                      // fault
  /* state (DS402) */
  STATE_CALIBRATING, // calibration
  STATE_COUNT // state (check，)
} MotorState;
/*  (CANopen 6040h) */
typedef union {
  uint16_t word;
  struct {
    uint16_t switch_on : 1;        // bit0:
    uint16_t enable_voltage : 1;   // bit1: enablevoltage
    uint16_t quick_stop : 1;       // bit2: stop
    uint16_t enable_operation : 1; // bit3: enable
    uint16_t reserved1 : 3;        // bit4-6:
    uint16_t fault_reset : 1;      // bit7: faultreset
    uint16_t halt : 1;             // bit8: stop (calibration)
    uint16_t reserved2 : 7;        // bit9-15:
  } bits;
} Controlword;
/* state (CANopen 6041h) */
typedef union {
  uint16_t word;
  struct {
    uint16_t ready_to_switch_on : 1; // bit0:
    uint16_t switched_on : 1;        // bit1:
    uint16_t operation_enabled : 1;  // bit2: enable
    uint16_t fault : 1;              // bit3: fault
    uint16_t voltage_enabled : 1;    // bit4: voltageenable
    uint16_t quick_stop : 1;         // bit5: stop
    uint16_t switch_on_disabled : 1; // bit6:
    uint16_t warning : 1;            // bit7: warning
    uint16_t manufacturer1 : 1;      // bit8:  (calibration)
    uint16_t remote : 1;             // bit9:
    uint16_t target_reached : 1;     // bit10: target
    uint16_t internal_limit : 1;     // bit11: limit
    uint16_t reserved : 4;           // bit12-15:
  } bits;
} Statusword;
/* fault */
#define FAULT_HISTORY_SIZE 5
/* state */
typedef struct {
  MotorState current_state;    // state
  MotorState target_state;     // targetstate (RequestState )
  Controlword controlword;     //
  Statusword statusword;       // state
  uint32_t state_entry_time;   // state
  bool transition_in_progress; // state
  uint32_t active_fault_code;  // fault
  uint16_t prev_controlword;   //  ()
  bool (*pre_check_callback)(MotorState to_state); // statecheck
  bool auto_advance;           //  ( RequestState set)
  /* fault (per-instance) */
  uint32_t fault_history[FAULT_HISTORY_SIZE]; // fault
  uint8_t fault_history_index;                // fault
} StateMachine;
/**
 * @brief initstate
 * @param sm state
 */
void StateMachine_Init(StateMachine *sm);
/**
 * @brief updatestate ()
 * @param sm state
 */
void StateMachine_Update(StateMachine *sm);
/**
 * @brief state
 * @param sm state
 * @param target_state targetstate
 * @return true=, false=
 */
bool StateMachine_RequestState(StateMachine *sm, MotorState target_state);
/**
 * @brief set
 * @param sm state
 * @param controlword
 */
void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword);
/**
 * @brief getstate
 * @param sm state
 * @return state
 */
uint16_t StateMachine_GetStatusword(const StateMachine *sm);
/**
 * @brief getstate
 * @param sm state
 * @return state
 */
MotorState StateMachine_GetState(const StateMachine *sm);
/**
 * @brief faultstate
 * @param sm state
 * @param fault_code fault
 */
void StateMachine_EnterFault(StateMachine *sm, uint32_t fault_code);
/**
 * @brief fault
 * @param sm state
 * @return true=, false=
 */
bool StateMachine_ClearFault(StateMachine *sm);
/**
 * @brief setstatecheck
 * @param sm state
 * @param callback
 */
void StateMachine_SetPreCheckCallback(StateMachine *sm,
                                      bool (*callback)(MotorState to_state));
#endif /* FSM_H */
