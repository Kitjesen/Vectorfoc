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
 * @file svpwm.c
 * @brief Space-vector PWM modulation with midpoint injection.
 */
#include "svpwm.h"
#include "math_common.h"
#include <stddef.h>

int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc) {
  if (Ta == NULL || Tb == NULL || Tc == NULL) {
    return SVPWM_STATUS_INVALID_INPUT;
  }

  if (!isfinite(Vbus) || Vbus < 1.0f || !isfinite(Valpha) ||
      !isfinite(Vbeta)) {
    *Ta = 0.5f;
    *Tb = 0.5f;
    *Tc = 0.5f;
    return SVPWM_STATUS_INVALID_INPUT;
  }

  /* Valpha/Vbeta are physical phase-to-neutral voltage references.  The
   * midpoint-injection duty equation therefore normalizes phase voltage by
   * Vbus directly.  Dividing by 2/3*Vbus would amplify the request by 1.5x. */
  float mod_alpha = Valpha / Vbus;
  float mod_beta = Vbeta / Vbus;

  float Va = mod_alpha;
  float Vb = -0.5f * mod_alpha + MATH_SQRT3_BY_2 * mod_beta;
  float Vc = -0.5f * mod_alpha - MATH_SQRT3_BY_2 * mod_beta;

  float Vmax = Va;
  if (Vb > Vmax)
    Vmax = Vb;
  if (Vc > Vmax)
    Vmax = Vc;

  float Vmin = Va;
  if (Vb < Vmin)
    Vmin = Vb;
  if (Vc < Vmin)
    Vmin = Vc;

  int overmodulation = (Vmax - Vmin > 1.0f);
  if (overmodulation) {
    float scale = 1.0f / (Vmax - Vmin);
    Va *= scale;
    Vb *= scale;
    Vc *= scale;
    Vmax *= scale;
    Vmin *= scale;
  }

  float Vcom = 0.5f * (Vmax + Vmin);
  *Ta = Math_Clamp(0.5f + Va - Vcom, 0.0f, 1.0f);
  *Tb = Math_Clamp(0.5f + Vb - Vcom, 0.0f, 1.0f);
  *Tc = Math_Clamp(0.5f + Vc - Vcom, 0.0f, 1.0f);

  return overmodulation ? SVPWM_STATUS_OVERMODULATION : SVPWM_STATUS_OK;
}
