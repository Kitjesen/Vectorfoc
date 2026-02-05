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
