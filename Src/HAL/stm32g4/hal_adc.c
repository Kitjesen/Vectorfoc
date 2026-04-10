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
 * @file hal_adc.c
 * @brief ADC HAL wrapper
 *
 * 层次设计：HAL 层不依赖 ALGO 层 motor_data，
 * 由 app_init.c 通过 MHAL_ADC_SetHandle() 注入 Motor_HAL_Handle_t 指针。
 */
#include "hal_adc.h"
#include "motor_hal_api.h"

static const Motor_HAL_Handle_t *s_hal = NULL;
static Motor_HAL_SensorData_t s_adc_cache = {0};

void MHAL_ADC_SetHandle(const void *hal_handle)
{
    s_hal = (const Motor_HAL_Handle_t *)hal_handle;
}

int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface)
{
    (void)interface;
    return 0;
}

int MHAL_ADC_Init(void) { return 0; }
int MHAL_ADC_Start(void) { return 0; }
int MHAL_ADC_Stop(void) { return 0; }

int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic)
{
    if (s_hal == NULL || s_hal->adc == NULL || s_hal->adc->update == NULL)
        return -1;
    s_hal->adc->update(&s_adc_cache);
    if (Ia) *Ia = s_adc_cache.i_a;
    if (Ib) *Ib = s_adc_cache.i_b;
    if (Ic) *Ic = s_adc_cache.i_c;
    return 0;
}

float MHAL_ADC_GetVbus(void)
{
    if (s_hal == NULL || s_hal->adc == NULL || s_hal->adc->update == NULL)
        return 0.0f;
    s_hal->adc->update(&s_adc_cache);
    return s_adc_cache.v_bus;
}

float MHAL_ADC_GetTemperature(void)
{
    if (s_hal == NULL || s_hal->adc == NULL || s_hal->adc->update == NULL)
        return 0.0f;
    s_hal->adc->update(&s_adc_cache);
    return s_adc_cache.temp;
}

int MHAL_ADC_CalibrateCurrent(void)
{
    if (s_hal == NULL || s_hal->adc == NULL ||
        s_hal->adc->calibrate_offsets == NULL)
        return -1;
    s_hal->adc->calibrate_offsets();
    return 0;
}
