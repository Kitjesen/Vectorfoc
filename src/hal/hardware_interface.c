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

#include "hardware_interface.h"
#ifdef USE_HAL_DRIVER
#include "board_configuration.h"
#include "main.h"
/* ADC temperature reading */
#endif
/**
 * @brief get
 */
uint32_t HAL_GetSystemTick(void) {
#ifdef USE_HAL_DRIVER
  return HAL_GetTick();
#else
  /* HAL， */
  static volatile uint32_t tick_counter = 0;
  return tick_counter;
#endif
}
/**
 * @brief
 */
void HAL_Delay(uint32_t ms) {
#ifdef USE_HAL_DRIVER
  uint32_t start = HAL_GetSystemTick();
  while ((HAL_GetSystemTick() - start) < ms) {
  }
#else
  /* （） */
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP();
  }
#endif
}
/**
 * @brief
 */
uint32_t HAL_EnterCritical(void) {
#ifdef USE_HAL_DRIVER
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
#else
  return 0;
#endif
}
/**
 * @brief
 */
void HAL_ExitCritical(uint32_t prev_state) {
#ifdef USE_HAL_DRIVER
  __set_PRIMASK(prev_state);
#else
  (void)prev_state;
#endif
}
/**
 * @brief
 * @note  0xAAAA
 */
void HAL_WatchdogFeed(void) {
#ifdef USE_HAL_DRIVER
  IWDG->KR = 0xAAAA;
#else
  /* mode： */
#endif
}
