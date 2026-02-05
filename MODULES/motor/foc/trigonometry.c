/**
 * @file trigonometry.c
 * @brief 三角函数快速计算实现
 */

#include "trigonometry.h"
#include "math_common.h"

void Trig_FastSinCos(float angle, float *sin_val, float *cos_val)
{
    /* 归一化角度到[-π, π] */
    angle = Math_NormalizeAngle(angle);
    
    /* 使用泰勒级数近似计算sin */
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
    
    /* 计算cos: cos(x) = sin(x + π/2) */
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
