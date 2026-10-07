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
 * @file communication_task.h
 * @brief CAN processing, deferred parameter persistence and status reporting.
 */
#ifndef COMMUNICATION_TASK_H
#define COMMUNICATION_TASK_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void CommTask_Init(void);                   // init
void CommTask_Process(void);
void CommTask_SetReportEnabled(bool enable);
/**
 * @brief Reserve Flash-save maintenance before mutating persistent state.
 * @return true when the maintenance lease is held for this save request.
 */
bool CommTask_BeginScheduledSave(void);
/** @brief Queue the Flash save after CommTask_BeginScheduledSave succeeds. */
void CommTask_CommitScheduledSave(void);
/** @brief Release an uncommitted save reservation without queuing Flash I/O. */
void CommTask_CancelScheduledSave(void);
/** @brief Reserve maintenance and queue a save with no additional mutation. */
bool CommTask_RequestScheduledSave(void);
#ifdef __cplusplus
}
#endif
#endif /* COMMUNICATION_TASK_H */
