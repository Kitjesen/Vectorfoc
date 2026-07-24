#ifndef TEST_APP_INIT_BINDING_MOCKS_SAFETY_CONTROL_H
#define TEST_APP_INIT_BINDING_MOCKS_SAFETY_CONTROL_H
#include "fault_detection.h"
#include "fsm.h"
#include <stdbool.h>
#include <stdint.h>
#ifndef VECTORFOC_MOTOR_DATA_TYPEDEF
#define VECTORFOC_MOTOR_DATA_TYPEDEF
 typedef struct MOTOR_DATA_s MOTOR_DATA;
#endif
 void Safety_Init(const SafetyConfig *config);
 void Safety_TriggerFault(uint32_t fault_bits, MOTOR_DATA *motor, StateMachine *fsm);
 bool Safety_HasActiveFault(void);
#endif
