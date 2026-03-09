/**
 * @file    main.h
 * @brief   Mock main.h for PC test environment
 */

#ifndef MOCK_MAIN_H
#define MOCK_MAIN_H

#ifdef TEST_ENV

#include "mock_hal_types.h"

/* Prevent inclusion of real STM32 headers */
#define __STM32G4xx_HAL_H
#define __STM32G4XX_HAL_CONF_H
#define __STM32G431xx_H
#define CMSIS_COMPILER_H

#else
/* Real hardware build */
#include "stm32g4xx_hal.h"
#endif

#endif /* MOCK_MAIN_H */
