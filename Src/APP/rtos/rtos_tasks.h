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
 * @file rtos_tasks.h
 * @brief RTOS
 */
#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief  - USB/VOFA (1kHz)
 */
void StartDefaultTask(void const *argument);
/**
 * @brief protection - safety (200Hz)
 */
void StartGuardTask(void const *argument);
/**
 * @brief  - CAN (500Hz)
 */
void StartCustomTask(void const *argument);
#ifdef __cplusplus
}
#endif
#endif /* RTOS_TASKS_H */
