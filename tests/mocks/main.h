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
