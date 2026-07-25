#ifndef MOTOR_HAL_XSTAR_MOCK_BSP_ADC_H
#define MOTOR_HAL_XSTAR_MOCK_BSP_ADC_H

#include "stm32g4xx_hal.h"

#define adc1_channel 4u
#define adc1_samples 1u
#define adc2_channel 1u
#define adc2_samples 1u

extern uint16_t adc1_dma_value[adc1_samples][adc1_channel];
extern uint16_t adc2_dma_value[adc2_samples][adc2_channel];

#endif
