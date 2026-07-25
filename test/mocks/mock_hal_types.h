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

#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif

/* Mock HAL types */
typedef struct {
    void *Instance;
} SPI_HandleTypeDef;

typedef struct {
    void *Instance;
} TIM_HandleTypeDef;

typedef struct {
    uint32_t ISR;
    uint32_t IER;
    uint32_t JDR1;
    uint32_t JDR2;
    uint32_t JDR3;
    uint32_t JDR4;
} ADC_TypeDef;

typedef struct {
    ADC_TypeDef *Instance;
    uint32_t ErrorCode;
    uint32_t Flags;
} ADC_HandleTypeDef;

typedef struct { void *Instance; } DMA_HandleTypeDef;
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { void *Instance; } PCD_HandleTypeDef;
#ifndef TEST_UART_HANDLE_TYPEDEF
#define TEST_UART_HANDLE_TYPEDEF
typedef struct { void *Instance; } UART_HandleTypeDef;
#endif

#define ADC_IT_RDY           (1u << 0)
#define ADC_IT_EOSMP         (1u << 1)
#define ADC_IT_EOC           (1u << 2)
#define ADC_IT_EOS           (1u << 3)
#define ADC_IT_OVR           (1u << 4)
#define ADC_IT_JEOC          (1u << 5)
#define ADC_IT_JEOS          (1u << 6)
#define ADC_IT_AWD1          (1u << 7)
#define ADC_IT_AWD2          (1u << 8)
#define ADC_IT_AWD3          (1u << 9)
#define ADC_IT_JQOVF         (1u << 10)

#define ADC_FLAG_RDY         ADC_IT_RDY
#define ADC_FLAG_EOSMP       ADC_IT_EOSMP
#define ADC_FLAG_EOC         ADC_IT_EOC
#define ADC_FLAG_EOS         ADC_IT_EOS
#define ADC_FLAG_OVR         ADC_IT_OVR
#define ADC_FLAG_JEOC        ADC_IT_JEOC
#define ADC_FLAG_JEOS        ADC_IT_JEOS
#define ADC_FLAG_AWD1        ADC_IT_AWD1
#define ADC_FLAG_AWD2        ADC_IT_AWD2
#define ADC_FLAG_AWD3        ADC_IT_AWD3
#define ADC_FLAG_JQOVF       ADC_IT_JQOVF

typedef void GPIO_TypeDef;

/* Mock HAL status */
typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *hfdcan);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);

/* Mock CMSIS intrinsics — defined as static inline to satisfy C99 strict mode */
static inline void __NOP(void) {}
static inline void __WFI(void) {}
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }

/* Mock ARM DSP */
static inline float arm_sin_f32(float x) { return sinf(x); }
static inline float arm_cos_f32(float x) { return cosf(x); }

/* HAL_GetTick: declared but not defined here.
 * Each test translation unit that needs it must provide its own definition,
 * OR link against a stub .c that provides it.
 * This avoids static-inline vs extern-declaration conflicts.
 */
uint32_t HAL_GetTick(void);

/* Mock GPIO */
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
static inline void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState s) {
    (void)port; (void)pin; (void)s;
}

#endif /* MOCK_COMMON_H */
