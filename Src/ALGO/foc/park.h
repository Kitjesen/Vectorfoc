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
 * @file    park.h
 * @brief   Park transform (αβ ↔ dq).
 * @details Pure algorithm, no hardware dependency.
 */

#ifndef FOC_PARK_H
#define FOC_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Park transform: αβ → dq.
 * @param  Ialpha  [A] Alpha-axis current.
 * @param  Ibeta   [A] Beta-axis current.
 * @param  theta   [rad] Electrical angle.
 * @param  Id      [out] [A] D-axis current.
 * @param  Iq      [out] [A] Q-axis current.
 */
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id,
                    float *Iq);

/**
 * @brief  Inverse Park transform: dq→ αβ.
 * @param  Vd      [V] D-axis voltage.
 * @param  Vq      [V] Q-axis voltage.
 * @param  theta   [rad] Electrical angle.
 * @param  Valpha  [out] [V] Alpha-axis voltage.
 * @param  Vbeta   [out] [V] Beta-axis voltage.
 */
void Park_Inverse(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PARK_H */
