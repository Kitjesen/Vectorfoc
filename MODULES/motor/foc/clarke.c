/**
 * @file clarke.c
 * @brief Clarke坐标变换实现
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
