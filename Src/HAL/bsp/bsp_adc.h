#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "common.h"
#include "adc.h"
#include "tim.h"
#define ADC_INJECTED_ENABLE 1 //  ADC mode
#define adc1_samples 5                         // sample
#define adc1_channel 1                         // sample
#define adc1_length adc1_samples *adc1_channel //
#define adc2_samples 5                         // sample
#define adc2_channel 1                         // sample
#define adc2_length adc2_samples *adc2_channel //
/* [FIX] DMA  volatile
 *  DMA ，CPU  */
extern volatile uint16_t adc1_dma_value[adc1_samples][adc1_channel];
extern volatile uint16_t adc2_dma_value[adc2_samples][adc2_channel];
void adc_bsp_init(void);
#endif
