// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file board_adc.c
 * @brief ADC hardware abstraction layer implementation
 */
#include "board_adc.h"
#include "board_configuration.h"
#include "error_manager.h"
#include "error_types.h"
/* ADC DMA (volatile: DMA) */
volatile uint16_t adc1_dma_value[adc1_samples][adc1_channel];
volatile uint16_t adc2_dma_value[adc2_samples][adc2_channel];
/**
 * @brief 启动 ADC
 *
 * 当前硬件适配使用 ADC1 注入采样和 ADC2 温度 DMA。
 */
/**
 * @brief ADC 操作辅助宏，失败时上报错误并返回；失败的 ADC 不能参与启动就绪判定。
 */
#define ADC_CHECK(expr) \
    do { \
        if ((expr) != HAL_OK) { \
            ERROR_REPORT(ERROR_HW_ADC_INIT, #expr " failed"); \
            return -1; \
        } \
    } while (0)

int adc_bsp_init(void) {
  /* ADC1  + ADC2 regular DMA（）。 */
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&HW_ADC_CURRENT, ADC_SINGLE_ENDED));
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&HW_ADC_TEMP, ADC_SINGLE_ENDED));
  ADC_CHECK(HAL_ADCEx_InjectedStart_IT(&HW_ADC_CURRENT));
  ADC_CHECK(HAL_ADC_Start_DMA(&HW_ADC_TEMP, (uint32_t *)adc2_dma_value, adc2_length));
  return 0;
}
