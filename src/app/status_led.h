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

#ifndef LED_H
#define LED_H
#include "common.h"
#include "tim.h"
/*calcCCR*/
#define CODE_1 (140) // 1timer
#define CODE_0 (70)  // 0timer
#define LED_MAX_NUM 1 // LED，LED
/*LED*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_TypeDef;
void RGB_SetColor(uint8_t LedId, RGB_Color_TypeDef Color); // LED24（01）
void Reset_Load(void);                                     // 240，RESET_code
void RGB_SendArray(void);                                  // LED
void RGB_DisplayColor(RGB_Color_TypeDef color);            //
void RGB_DisplayColorById(uint8_t color_id);               //
void RGB_DMA_CompleteCallback(void);                       // DMAdone
#endif
