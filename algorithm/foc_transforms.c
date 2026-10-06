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

#include "foc_transforms.h"

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

void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id,
                    float *Iq) {
  float sin_val, cos_val;
  Trig_FastSinCos(theta, &sin_val, &cos_val);

  *Id = Ialpha * cos_val + Ibeta * sin_val;
  *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}

void Park_Inverse(float Vd, float Vq, float theta, float *Valpha,
                  float *Vbeta) {
  float sin_val, cos_val;
  Trig_FastSinCos(theta, &sin_val, &cos_val);

  *Valpha = Vd * cos_val - Vq * sin_val;
  *Vbeta = Vd * sin_val + Vq * cos_val;
}

int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc) {
  // [FIX] 添加 Vbus 有效性检查，避免除零
  if (Vbus < 1.0f) {
    *Ta = 0.5f;
    *Tb = 0.5f;
    *Tc = 0.5f;
    return -1;  // 返回错误码表示 Vbus 无效
  }

  /* Valpha/Vbeta are volts. Duty differences reproduce phase-voltage
   * differences divided by Vbus, so the DC bus itself is the scale factor.
   * The 2/3 factor belongs to the inverse Clarke/space-vector relationship;
   * applying it here would amplify the requested voltage by 1.5x. */
  float mod_alpha = Valpha / Vbus;
  float mod_beta = Vbeta / Vbus;
  /* phase voltage expressed as a fraction of the DC bus */
  float Va = mod_alpha;
  float Vb = -0.5f * mod_alpha + MATH_SQRT3_BY_2 * mod_beta;
  float Vc = -0.5f * mod_alpha - MATH_SQRT3_BY_2 * mod_beta;
  /*  */
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
  /* voltage() */
  float Vcom = 0.5f * (Vmax + Vmin);
  /* Check for over-modulation and apply scaling. */
  int status = 0;
  if (Vmax - Vmin > 1.0f) {
    status = 1;
    float scale = 1.0f / (Vmax - Vmin);
    Va *= scale;
    Vb *= scale;
    Vc *= scale;
    // Re-calculate Vcom with scaled voltages
    Vmax *= scale;
    Vmin *= scale;
    Vcom = 0.5f * (Vmax + Vmin);
  }
  /* Calculate final duty cycles */
  *Ta = 0.5f + Va - Vcom;
  *Tb = 0.5f + Vb - Vcom;
  *Tc = 0.5f + Vc - Vcom;
  /* Clamp to [0, 1] as a final safety */
  *Ta = Math_Clamp(*Ta, 0.0f, 1.0f);
  *Tb = Math_Clamp(*Tb, 0.0f, 1.0f);
  *Tc = Math_Clamp(*Tc, 0.0f, 1.0f);
  return status;
}

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
