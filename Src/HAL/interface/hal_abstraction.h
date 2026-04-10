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
 * @file hal_abstraction.h
 * @brief HAL -
 * @version 1.0
 * @date 2026-01-20
 *
 * phase，HAL
 */
#ifndef HAL_ABSTRACTION_H
#define HAL_ABSTRACTION_H
#include "stdint.h"
#include "stdbool.h"
#include "protocol_types.h"
/**
 * @brief get（）
 * @return start
 *
 * @note STM32 HALHAL_GetTick()
 */
uint32_t HAL_GetSystemTick(void);
/**
 * @brief get（，）
 * @return start
 */
uint32_t HAL_GetMicroseconds(void);
/**
 * @brief CAN
 * @param frame CAN
 * @return true=, false=（error）
 *
 * @note CAN ID（/）RTR
 */
bool HAL_CAN_Transmit(const CAN_Frame *frame);
/**
 * @brief checkCAN
 * @return true=, false=
 */
bool HAL_CAN_IsTxMailboxAvailable(void);
/**
 * @brief gettemperature
 * @return temperature（）
 *
 * @note temperature，
 */
float HAL_GetTemperature(void);
/**
 * @brief （）
 * @param ms
 */
void HAL_Delay(uint32_t ms);
/**
 * @brief （disableinterrupt）
 * @return interruptstate（）
 */
uint32_t HAL_EnterCritical(void);
/**
 * @brief （interrupt）
 * @param prev_state interruptstate
 */
void HAL_ExitCritical(uint32_t prev_state);
/**
 * @brief
 * @note interrupt，reset
 *       register
 */
void HAL_WatchdogFeed(void);
#endif /* HAL_ABSTRACTION_H */
