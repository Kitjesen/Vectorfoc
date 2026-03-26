/**
 * @file isr_foc.h
 * @brief FOCinterrupt
 */
#ifndef ISR_FOC_H
#define ISR_FOC_H
#include "stm32g4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief ADCinterrupt (20kHz FOC)
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
#ifdef __cplusplus
}
#endif
#endif /* ISR_FOC_H */
