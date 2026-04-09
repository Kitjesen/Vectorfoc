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
