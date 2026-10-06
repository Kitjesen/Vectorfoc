#ifndef STARTUP_TEST_ADC_H
#define STARTUP_TEST_ADC_H
#ifdef STARTUP_TEST
#include "mock_hal_types.h"
#endif
#include <stdint.h>
#ifndef MOCK_COMMON_H
typedef struct { uint32_t JDR1, JDR2, JDR3, JDR4; } StartupADCRegisters;
typedef struct { StartupADCRegisters *Instance; } ADC_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
#endif
extern ADC_HandleTypeDef hadc1, hadc2;
#define ADC_SINGLE_ENDED 0
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *, uint32_t *, uint32_t);
uint32_t HAL_GetTick(void);
#endif
