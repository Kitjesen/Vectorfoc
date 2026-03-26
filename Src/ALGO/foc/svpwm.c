/**
 * @file svpwm.c
 * @brief modulation
 */
#include "svpwm.h"
#include "math_common.h"
int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc) {
  // [FIX] 添加 Vbus 有效性检查，避免除零
  if (Vbus < 1.0f) {
    *Ta = 0.5f;
    *Tb = 0.5f;
    *Tc = 0.5f;
    return -1;  // 返回错误码表示 Vbus 无效
  }
  
  /* voltage */
  float mod_alpha = Valpha / (Vbus * MATH_2_BY_3);
  float mod_beta = Vbeta / (Vbus * MATH_2_BY_3);
  /* phasevoltage */
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
  /* Check for over-modulation and apply scaling */
  if (Vmax - Vmin > 1.0f) {
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
  return 0;
}
