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
 * @file trigonometry.c
 * @brief calc
 */
#include "trigonometry.h"
#include "math_common.h"
void Trig_FastSinCos(float angle, float *sin_val, float *cos_val)
{
    /* angle[-π, π] */
    angle = Math_NormalizeAngle(angle);
    /* calcsin */
    if (angle < 0.0f)
    {
        *sin_val = 1.27323954f * angle + 0.405284735f * angle * angle;
        if (*sin_val < 0.0f)
            *sin_val = 0.225f * (*sin_val * -*sin_val - *sin_val) + *sin_val;
        else
            *sin_val = 0.225f * (*sin_val * *sin_val - *sin_val) + *sin_val;
    }
    else
    {
        *sin_val = 1.27323954f * angle - 0.405284735f * angle * angle;
        if (*sin_val < 0.0f)
            *sin_val = 0.225f * (*sin_val * -*sin_val - *sin_val) + *sin_val;
        else
            *sin_val = 0.225f * (*sin_val * *sin_val - *sin_val) + *sin_val;
    }
    /* calccos: cos(x) = sin(x + π/2) */
    angle += 0.5f * MATH_PI;
    if (angle > MATH_PI)
        angle -= MATH_2PI;
    if (angle < 0.0f)
    {
        *cos_val = 1.27323954f * angle + 0.405284735f * angle * angle;
        if (*cos_val < 0.0f)
            *cos_val = 0.225f * (*cos_val * -*cos_val - *cos_val) + *cos_val;
        else
            *cos_val = 0.225f * (*cos_val * *cos_val - *cos_val) + *cos_val;
    }
    else
    {
        *cos_val = 1.27323954f * angle - 0.405284735f * angle * angle;
        if (*cos_val < 0.0f)
            *cos_val = 0.225f * (*cos_val * -*cos_val - *cos_val) + *cos_val;
        else
            *cos_val = 0.225f * (*cos_val * *cos_val - *cos_val) + *cos_val;
    }
}
