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
 * @file bsp_adc.c
 * @brief ADC hardware abstraction layer implementation
 */
#include "bsp_adc.h"
#include "board_config.h"
#include "error_manager.h"
#include "error_types.h"
/* ADC DMA (volatile: DMA) */
volatile uint16_t adc1_dma_value[adc1_samples][adc1_channel];
volatile uint16_t adc2_dma_value[adc2_samples][adc2_channel];
/**
 * @brief 启动 ADC
 *
 * X-STAR-S：ADC1(4ch注入IT) + ADC2(1ch注入轮询)，均由 TIM1_CC4 触发
 * VectorFOC：ADC1(注入IT) + ADC2(regular DMA 温度)
 */
/**
 * @brief ADC 操作辅助宏，失败时上报错误码并继续（不 hang 系统，
 *        避免单点 HAL 失败导致整机死锁，但故障会被记录到 ErrorManager）
 */
#define ADC_CHECK(expr) \
    do { \
        if ((expr) != HAL_OK) { \
            ERROR_REPORT(ERROR_HW_ADC_INIT, #expr " failed"); \
        } \
    } while (0)

void adc_bsp_init(void) {
#ifdef BOARD_XSTAR
  /* X-STAR-S：两路 ADC 均为注入模式，无 DMA */
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED));
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED));
  /* ADC1：注入完成中断（驱动 FOC ISR） */
  ADC_CHECK(HAL_ADCEx_InjectedStart_IT(&hadc1));
  /* ADC2：注入模式启动，结果由 FOC ISR 直接读 JDR1，不需要中断 */
  ADC_CHECK(HAL_ADCEx_InjectedStart(&hadc2));
#elif ADC_INJECTED_ENABLE
  /* VectorFOC：ADC1 注入IT + ADC2 regular DMA（温度） */
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&HW_ADC_CURRENT, ADC_SINGLE_ENDED));
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&HW_ADC_TEMP, ADC_SINGLE_ENDED));
  ADC_CHECK(HAL_ADCEx_InjectedStart_IT(&HW_ADC_CURRENT));
  ADC_CHECK(HAL_ADC_Start_DMA(&HW_ADC_TEMP, (uint32_t *)adc2_dma_value, adc2_length));
#else
  ADC_CHECK(HAL_ADCEx_Calibration_Start(&HW_ADC_CURRENT, ADC_SINGLE_ENDED));
  ADC_CHECK(HAL_ADC_Start_DMA(&HW_ADC_CURRENT, (uint32_t *)adc1_dma_value, adc1_length));
#endif
}
