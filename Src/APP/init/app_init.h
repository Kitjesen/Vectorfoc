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
 * @file app_init.h
 * @brief init
 */
#ifndef APP_INIT_H
#define APP_INIT_H
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief init ( main() )
 * @note init:
 *   1. BSP (DWT, Log, ADC, PWM)
 *   2. Safety & Detection
 *   3. Parameter System
 *   4. CAN & Protocol
 *   5. State Machine
 *   6. Motor
 */
void App_Init(void);
#ifdef __cplusplus
}
#endif
#endif /* APP_INIT_H */
