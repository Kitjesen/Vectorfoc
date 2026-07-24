#ifndef TEST_APP_INIT_BINDING_MOCKS_FSM_H
#define TEST_APP_INIT_BINDING_MOCKS_FSM_H
#include <stdbool.h>
typedef enum { STATE_NOT_READY_TO_SWITCH_ON = 0, STATE_SWITCH_ON_DISABLED, STATE_READY_TO_SWITCH_ON, STATE_SWITCHED_ON, STATE_OPERATION_ENABLED, STATE_QUICK_STOP_ACTIVE, STATE_FAULT_REACTION_ACTIVE, STATE_FAULT, STATE_CALIBRATING, STATE_COUNT } MotorState;
typedef struct { bool (*pre_check_callback)(MotorState to_state); } StateMachine;
void StateMachine_Init(StateMachine *sm);
void StateMachine_SetPreCheckCallback(StateMachine *sm, bool (*callback)(MotorState to_state));
#endif