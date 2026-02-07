/**
 * @file bsp_adc.c
 * @brief ADC hardware abstraction layer implementation
 */

#include "bsp_adc.h"
#include "board_config.h"

/* ADC DMA数据存储缓冲区 */
uint16_t adc1_dma_value[adc1_samples][adc1_channel];
uint16_t adc2_dma_value[adc2_samples][adc2_channel];

/**
 * @brief 初始化ADC硬件
 * @note 包含ADC校准和DMA/注入模式启动
 */
void adc_bsp_init(void) {
#if ADC_INJECTED_ENABLE
  /* Calibrate both ADCs */
  HAL_ADCEx_Calibration_Start(&HW_ADC_CURRENT, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&HW_ADC_TEMP, ADC_SINGLE_ENDED);

  /* Start current ADC in injected interrupt mode, temp ADC in DMA mode */
  HAL_ADCEx_InjectedStart_IT(&HW_ADC_CURRENT);
  HAL_ADC_Start_DMA(&HW_ADC_TEMP, (uint32_t *)adc2_dma_value, adc2_length);

#else /* Regular DMA mode */
  HAL_ADCEx_Calibration_Start(&HW_ADC_CURRENT, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&HW_ADC_CURRENT, (uint32_t *)adc1_dma_value, adc1_length);
#endif
}
