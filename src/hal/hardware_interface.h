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

#ifndef HAL_ABSTRACTION_H
#define HAL_ABSTRACTION_H
#include <stdint.h>

/* Monotonic time since boot. */
uint32_t HAL_GetSystemTick(void);      /* milliseconds */
void HAL_Delay(uint32_t ms);

/* Save and restore the interrupt mask; callers must pair these operations. */
uint32_t HAL_EnterCritical(void);
void HAL_ExitCritical(uint32_t prev_state);
void HAL_WatchdogFeed(void);
#endif /* HAL_ABSTRACTION_H */
