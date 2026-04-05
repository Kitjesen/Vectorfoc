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
 * @file clarke.c
 * @brief Clarke
 */
#include "clarke.h"
#include "math_common.h"
void Clarke_Transform(float Ia, float Ib, float Ic,
                      float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (Ib - Ic) * MATH_ONE_BY_SQRT3;
}
void Clarke_Inverse(float Valpha, float Vbeta,
                    float *Va, float *Vb, float *Vc)
{
    *Va = Valpha;
    *Vb = -0.5f * Valpha + MATH_SQRT3_BY_2 * Vbeta;
    *Vc = -0.5f * Valpha - MATH_SQRT3_BY_2 * Vbeta;
}
