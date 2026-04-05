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

#ifndef BSP_INIT_H
#define BSP_INIT_H
#include "bsp_dwt.h"
/**
 * @brief bspinit,initbsp,init
 *        start,RobotoInit()
 *
 * @note CANinit,init
 */
/* 根据实际时钟初始化 DWT，支持 168MHz（VectorFOC）和 170MHz（X-STAR-S） */
void BSPInit()
{
  DWT_Init(SystemCoreClock / 1000000UL);
}
#endif
