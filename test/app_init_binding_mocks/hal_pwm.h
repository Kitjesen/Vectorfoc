#ifndef TEST_APP_INIT_BINDING_MOCKS_HAL_PWM_H
#define TEST_APP_INIT_BINDING_MOCKS_HAL_PWM_H
#include "motor_hal_api.h"
int MHAL_PWM_Bind(const Motor_HAL_PwmInterface_t *interface);
int MHAL_PWM_StartSampling(void);
int MHAL_PWM_Disable(void);
#endif