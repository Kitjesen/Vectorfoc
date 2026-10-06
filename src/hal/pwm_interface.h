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

#ifndef HAL_PWM_H
#define HAL_PWM_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Start ADC sampling without enabling any of the six power outputs. */
int MHAL_PWM_StartSampling(void);
int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc);
int MHAL_PWM_Enable(void);
/* Leave the ADC sampling timebase running when power outputs are disabled. */
int MHAL_PWM_Disable(void);
/* Active low-side braking, permitted only in an accepted powered state. */
int MHAL_PWM_Brake(void);
#ifdef __cplusplus
}
#endif
#endif
