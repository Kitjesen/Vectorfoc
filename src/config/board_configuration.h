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
 * @file    board_configuration.h
 * @brief   当前固件硬件配置入口。
 *
 * 算法核心位于 firmware/algorithm，不包含本文件。只有固件适配层需要
 * 看到这些 STM32 和功率级宏；上层源文件统一通过此入口引用它们。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "boards/board_vectorfoc.h"

#endif /* BOARD_CONFIG_H */
