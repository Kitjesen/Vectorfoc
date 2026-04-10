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
 * @file    common.h
 * @brief   Mock common header for PC test environment
 */

#ifndef MOCK_COMMON_H
#define MOCK_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

/* Mock ARM math types */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define M_2PI (2.0f * M_PI)

/* Mock HAL types */
typedef struct {
    void *Instance;
} SPI_HandleTypeDef;

typedef struct {
    void *Instance;
} TIM_HandleTypeDef;

typedef struct {
    void *Instance;
} ADC_HandleTypeDef;

typedef void GPIO_TypeDef;

/* Mock HAL status */
typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

/* Mock CMSIS intrinsics */
#define __NOP() ((void)0)
#define __WFI() ((void)0)
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

/* Mock ARM DSP */
static inline float arm_sin_f32(float x) { return sinf(x); }
static inline float arm_cos_f32(float x) { return cosf(x); }

#endif /* MOCK_COMMON_H */
