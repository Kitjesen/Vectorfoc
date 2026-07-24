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
 * @file app_comm_bootstrap.h
 * @brief Startup composition for the CAN transport, protocol and fault bridge.
 */
#ifndef APP_COMM_BOOTSTRAP_H
#define APP_COMM_BOOTSTRAP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the CAN communication stack in application startup order.
 *
 * This must run after early parameter recovery so the persisted CAN baud rate
 * and protocol type are available, and before the runtime-settings replay.
 */
bool AppComm_Bootstrap(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMM_BOOTSTRAP_H */
