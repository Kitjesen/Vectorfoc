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
 * @file hal_adc.h
 * @brief ADC ?
 * @note ?ュ
 */
#ifndef HAL_ADC_H
#define HAL_ADC_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief ADC ュ?
 */
typedef struct {
    /**
     * @brief ?ADC
     */
    void (*init)(void);
    /**
     * @brief  ADC 
     */
    void (*start)(void);
    /**
     * @brief  ADC 
     */
    void (*stop)(void);
    /**
     * @brief
     * @param Ia a ?[A] ()
     * @param Ib b ?[A] ()
     * @param Ic c ?[A] ()
     */
    void (*get_current)(float *Ia, float *Ib, float *Ic);
    /**
     * @brief
     * @return  [V]
     */
    float (*get_vbus)(void);
    /**
     * @brief ╁
     * @return ╁ [C]
     */
    float (*get_temperature)(void);
    /**
     * @brief ″
     */
    void (*calibrate_current)(void);
} HAL_ADC_Interface_t;
/**
 * @brief ㄥ ADC ュ
 */
int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface);
/**
 * @brief ?ADC
 */
int MHAL_ADC_Init(void);
/**
 * @brief  ADC 
 */
int MHAL_ADC_Start(void);
/**
 * @brief  ADC 
 */
int MHAL_ADC_Stop(void);
/**
 * @brief
 */
int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic);
/**
 * @brief
 */
float MHAL_ADC_GetVbus(void);
/**
 * @brief ╁
 */
float MHAL_ADC_GetTemperature(void);
/**
 * @brief ″
 */
int MHAL_ADC_CalibrateCurrent(void);
#ifdef __cplusplus
}
#endif
#endif /* HAL_ADC_H */
