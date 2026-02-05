/**
 * @file svpwm.c
 * @brief 空间矢量脉宽调制实现
 */

#include "svpwm.h"
#include "math_common.h"

int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc) {
  /* 归一化电压 */
  float mod_alpha = Valpha / (Vbus * MATH_2_BY_3);
  float mod_beta = Vbeta / (Vbus * MATH_2_BY_3);

  /* 转换为三相电压 */
  float Va = mod_alpha;
  float Vb = -0.5f * mod_alpha + MATH_SQRT3_BY_2 * mod_beta;
  float Vc = -0.5f * mod_alpha - MATH_SQRT3_BY_2 * mod_beta;

  /* 找到最大值和最小值 */
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

  /* 中点电压(零序分量注入) */
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
