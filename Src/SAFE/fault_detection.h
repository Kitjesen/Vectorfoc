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
 * @file fault_detection.h
 * @brief fault -
 * @version 2.0
 * @date 2026-01-23
 */
#ifndef FAULT_DETECTION_H
#define FAULT_DETECTION_H
#include "fault_def.h"
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief init
 * @param config configparam (NULL)
 */
void Detection_Init(const DetectionConfig *config);
/**
 * @brief  (state)
 * @param motor motor (void* )
 * @return fault
 */
uint32_t Detection_Check(void *motor);
/**
 * @brief fault (fault，20kHz)
 * @param motor motor
 * @return fault
 * @note fault，1μs
 */
uint32_t Detection_Check_Fast(void *motor);
/**
 * @brief fault (fault，200Hz)
 * @param motor motor
 * @return fault
 * @note voltage、temperature、、CANtimeout，3μs
 */
uint32_t Detection_Check_Slow(void *motor);
/**
 * @brief state ()
 */
void Detection_Reset(void);
/**
 * @brief getstate ()
 */
const DetectionState *Detection_GetState(void);
/**
 * @brief updateCAN ()
 */
void Detection_FeedWatchdog(uint32_t timestamp);
/**
 * @brief 获取检测配置指针（运行时访问，推荐使用）
 * @return 配置指针（非 NULL）
 */
DetectionConfig *Detection_GetConfig(void);

/**
 * @brief 检测配置全局实例（仅供 param_table.c 静态初始化器使用）
 *
 * 嵌入式参数表需要在编译时取地址（.ptr = &s_config.field），
 * 因此无法通过 Detection_GetConfig() 替代。
 * 其他模块应通过 Detection_GetConfig() 获取指针，不得直接操作此变量。
 */
extern DetectionConfig s_config;
#endif /* FAULT_DETECTION_H */
