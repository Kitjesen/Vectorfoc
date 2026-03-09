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
