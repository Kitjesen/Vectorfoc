/**
 * @file isr_foc.h
 * @brief FOC中断接口声明
 */

#ifndef ISR_FOC_H
#define ISR_FOC_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADC注入通道中断回调 (20kHz FOC控制)
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* ISR_FOC_H */
