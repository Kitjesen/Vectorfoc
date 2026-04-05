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
 * @file    svpwm.h
 * @brief   Space Vector PWM (SVPWM) modulation.
 * @details Pure algorithm, no hardware dependency.
 */

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SVPWM modulation: αβ voltage → PWM duty cycles.
 * @param  Valpha  [V] Alpha-axis voltage reference.
 * @param  Vbeta   [V] Beta-axis voltage reference.
 * @param  Vbus    [V] Bus voltage.
 * @param  Ta      [out] [0~1] Phase A duty cycle.
 * @param  Tb      [out] [0~1] Phase B duty cycle.
 * @param  Tc      [out] [0~1] Phase C duty cycle.
 * @return 0 on success, -1 on over-modulation (voltage saturation).
 * @note   Uses midpoint injection method.
 */
int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVPWM_H */
