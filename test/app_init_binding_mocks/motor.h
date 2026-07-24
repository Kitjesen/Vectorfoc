#ifndef TEST_APP_INIT_BINDING_MOCKS_MOTOR_H
#define TEST_APP_INIT_BINDING_MOCKS_MOTOR_H
#include "fsm.h"
#include "motor_hal_api.h"
#include <stdbool.h>
#include <stdint.h>
 typedef struct { const Motor_HAL_Handle_t *hal; } MOTOR_COMPONENTS;
 typedef struct { int pole_pairs; } MOTOR_PARAMETERS;
 typedef struct { float position; float velocity; float phase_angle; float temperature; } MOTOR_FEEDBACK;
 typedef struct { float Ia; float Ib; float Ic; float Vbus; float theta_elec; } FOC_AlgorithmInput_t;
#ifndef VECTORFOC_MOTOR_DATA_TYPEDEF
#define VECTORFOC_MOTOR_DATA_TYPEDEF
 typedef struct MOTOR_DATA_s MOTOR_DATA;
#endif
 struct MOTOR_DATA_s { MOTOR_COMPONENTS components; MOTOR_PARAMETERS parameters; MOTOR_FEEDBACK feedback; FOC_AlgorithmInput_t algo_input; };
 extern MOTOR_DATA motor_data;
 extern StateMachine g_ds402_state_machine;
 uint8_t Motor_PreCalibCheck(MOTOR_DATA *motor, uint8_t *fail_mask);
 void Init_Motor_No_Calib(MOTOR_DATA *motor);
#endif
