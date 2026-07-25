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

#include "hal_abstraction.h"
#ifdef USE_HAL_DRIVER
#include "board_config.h"
#include "main.h"
/* ADC temperature reading */
#include "bsp_adc.h"
#include "motor_adc.h"
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
 * @brief get
 */
uint32_t HAL_GetMicroseconds(void) {
#ifdef USE_HAL_DRIVER
  /* DWT（Data Watchpoint and Trace） */
  return DWT->CYCCNT / (SystemCoreClock / 1000000);
#else
  /*  */
  return HAL_GetSystemTick() * 1000;
#endif
}
/**
 * @brief gettemperature
 * @return temperature（）
 *
 * @note ADC2_IN12(PB2)acquisitionNTCvoltage，temperature
 */
float HAL_GetTemperature(void) {
#ifdef USE_HAL_DRIVER
  /* ADC2temperature（0adc2_ch12）filter */
  uint16_t adc_raw = adc2_avg_filter(adc2_ch12);
  /* NTCactualtemperature */
  float temp;
  GetTempNtc(adc_raw, &temp);
  return temp;
#else
  return 25.0f;
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
