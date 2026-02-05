/**
 * @file    trigonometry.h
 * @brief   Fast trigonometric functions for real-time control.
 * @details Taylor series approximation (~5x faster than math.h, ~0.1% error).
 */

#ifndef FOC_TRIGONOMETRY_H
#define FOC_TRIGONOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Fast sin/cos calculation (Taylor series).
 * @param  angle    [rad] Angle.
 * @param  sin_val  [out] sin(angle).
 * @param  cos_val  [out] cos(angle).
 * @note   Accuracy: ~0.1% error, Speed: ~5x faster than math.h.
 */
void Trig_FastSinCos(float angle, float *sin_val, float *cos_val);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRIGONOMETRY_H */
