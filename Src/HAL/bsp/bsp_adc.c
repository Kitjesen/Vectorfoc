/**
 * @file bsp_adc.c
 * @brief ADC硬件抽象层实现
 */

#include "bsp_adc.h"

/* ADC DMA数据存储缓冲区 */
uint16_t adc1_dma_value[adc1_samples][adc1_channel];
uint16_t adc2_dma_value[adc2_samples][adc2_channel];

/**
 * @brief 初始化ADC硬件
 * @note 包含ADC校准和DMA/注入模式启动
 */
void adc_bsp_init(void) {
#if ADC_INJECTED_ENABLE
  // 1. ADC校准
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // 2. 启动ADC注入中断模式
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma_value, adc2_length);

#else // 使用常规DMA模式
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_value, adc1_length);
#endif
}
