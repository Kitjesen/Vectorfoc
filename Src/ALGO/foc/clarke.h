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
 * @file    clarke.h
 * @brief   Clarke transform (abc ↔ αβ).
 * @details Pure algorithm, no hardware dependency.
 */

#ifndef FOC_CLARKE_H
#define FOC_CLARKE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Clarke transform: abc → αβ.
 * @param  Ia      [A] Phase A current.
 * @param  Ib      [A] Phase B current.
 * @param  Ic      [A] Phase C current.
 * @param  Ialpha  [out] [A] Alpha-axis current.
 * @param  Ibeta   [out] [A] Beta-axis current.
 * @note   Assumes Ia + Ib + Ic = 0.
 */
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha,
                      float *Ibeta);

/**
 * @brief  Inverse Clarke transform: αβ → abc.
 * @param  Valpha  [V] Alpha-axis voltage.
 * @param  Vbeta   [V] Beta-axis voltage.
 * @param  Va      [out] [V] Phase A voltage.
 * @param  Vb      [out] [V] Phase B voltage.
 * @param  Vc      [out] [V] Phase C voltage.
 */
void Clarke_Inverse(float Valpha, float Vbeta, float *Va, float *Vb, float *Vc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CLARKE_H */
