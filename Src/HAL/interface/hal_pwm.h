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
 * @file hal_pwm.h
 * @brief PWM
 * @note  PWM ，
 */
#ifndef HAL_PWM_H
#define HAL_PWM_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief PWM
 *
 * driver， HAL
 */
typedef struct {
  /**
   * @brief init PWM
   */
  void (*init)(void);
  /**
   * @brief setphaseduty cycle
   * @param Ta a phaseduty cycle [0.0~1.0]
   * @param Tb b phaseduty cycle [0.0~1.0]
   * @param Tc c phaseduty cycle [0.0~1.0]
   */
  void (*set_duty)(float Ta, float Tb, float Tc);
  /**
   * @brief  PWM output
   */
  void (*enable)(void);
  /**
   * @brief disable PWM output
   */
  void (*disable)(void);
  /**
   * @brief set（）
   */
  void (*brake)(void);
  /**
   * @brief get PWM frequency
   * @return PWM frequency [Hz]
   */
  uint32_t (*get_frequency)(void);
  /**
   * @brief get PWM period
   * @return PWM period [us]
   */
  float (*get_period)(void);
} HAL_PWM_Interface_t;
/**
 * @brief  PWM
 *
 * driverinit，
 *
 * @param interface PWM
 * @return 0: , -1:
 */
int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface);
int MHAL_PWM_Init(void);
int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc);
int MHAL_PWM_Enable(void);
int MHAL_PWM_Disable(void);
int MHAL_PWM_Brake(void);
uint32_t MHAL_PWM_GetFrequency(void);
float MHAL_PWM_GetPeriod(void);
#ifdef __cplusplus
}
#endif
#endif /* HAL_PWM_H */
