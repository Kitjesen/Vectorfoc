// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/**
 * @file stm32g4xx_hal.h (TEST_ENV stub)
 */
#ifndef STM32G4XX_HAL_H
#define STM32G4XX_HAL_H

#include "mock_hal_types.h"   /* 所有 HAL 类型都在这里定义 */

/* 阻止嵌套 HAL 头 */
#define STM32G4xx_HAL_CONF_H
#define STM32G431xx_H

/* FDCAN / UART 仅在此占位（mock_hal_types.h 已有 SPI/TIM/ADC） */
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
#ifndef UART_HandleTypeDef
typedef struct { void *Instance; } UART_HandleTypeDef;
#endif

/* 常用 HAL 宏 */
#define __HAL_TIM_GET_AUTORELOAD(h)         (0u)
#define __HAL_TIM_SET_COMPARE(h, ch, val)   ((void)0)
#define HAL_TIM_PWM_Start(h, ch)            HAL_OK
#define HAL_TIM_PWM_Stop(h, ch)             HAL_OK
#define HAL_TIMEx_PWMN_Start(h, ch)         HAL_OK
#define HAL_TIMEx_PWMN_Stop(h, ch)          HAL_OK
#define HAL_TIM_PWM_Start_DMA(h,ch,buf,len) HAL_OK
#define HAL_TIM_PWM_Stop_DMA(h, ch)         ((void)0)
#define HAL_SPI_TransmitReceive(h,t,r,s,to) HAL_OK
#define HAL_RCC_DeInit()                    ((void)0)
#define HAL_DeInit()                        ((void)0)
#define HAL_NVIC_SystemReset()              ((void)0)
#define HAL_GetSystemTick()                 (0u)

/* TIM channel aliases */
#define TIM_CHANNEL_1  0u
#define TIM_CHANNEL_2  1u
#define TIM_CHANNEL_3  2u
#define TIM_CHANNEL_4  3u

/* SysTick / NVIC stubs */
typedef struct { uint32_t CTRL; uint32_t LOAD; uint32_t VAL; } SysTick_Type;
static SysTick_Type _SysTick_stub;
#define SysTick (&_SysTick_stub)

typedef struct { uint32_t ICER[8]; uint32_t ICPR[8]; } NVIC_Type;
static NVIC_Type _NVIC_stub;
#define NVIC (&_NVIC_stub)

#define __set_MSP(v) ((void)(v))
#define __DSB()      ((void)0)
#define __ISB()      ((void)0)
#define __IO         volatile

#endif /* STM32G4XX_HAL_H */
