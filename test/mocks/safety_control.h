#ifndef TEST_MOCK_SAFETY_CONTROL_H
#define TEST_MOCK_SAFETY_CONTROL_H

#include "fsm.h"
#include <stdint.h>

uint32_t Safety_GetActiveFaultBits(void);
uint32_t Safety_GetLastFaultTime(void);
void Safety_ClearFaults(StateMachine *fsm);

#endif
