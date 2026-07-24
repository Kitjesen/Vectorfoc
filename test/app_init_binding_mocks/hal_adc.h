#ifndef TEST_APP_INIT_BINDING_MOCKS_HAL_ADC_H
#define TEST_APP_INIT_BINDING_MOCKS_HAL_ADC_H
#include "motor_hal_api.h"
int MHAL_ADC_Bind(const Motor_HAL_AdcInterface_t *interface);
int MHAL_ADC_CalibrateCurrent(void);
#endif