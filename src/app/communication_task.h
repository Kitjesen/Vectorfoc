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
 * @brief CAN task controls; processing runs in task context, never in an ISR.
 */
#ifndef TASK_COMM_H
#define TASK_COMM_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/** Drain queued frames, service deferred Flash saves and publish due reports. */
void CommTask_Process(void);
/** Enable or disable the existing 100 Hz CAN motor feedback stream. */
void CommTask_SetReportEnabled(bool enable);

#ifdef __cplusplus
}
#endif
#endif /* TASK_COMM_H */
