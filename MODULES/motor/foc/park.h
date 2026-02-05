/**
 * @file    park.h
 * @brief   Park transform (αβ ↔ dq).
 * @details Pure algorithm, no hardware dependency.
 */

#ifndef FOC_PARK_H
#define FOC_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Park transform: αβ → dq.
 * @param  Ialpha  [A] Alpha-axis current.
 * @param  Ibeta   [A] Beta-axis current.
 * @param  theta   [rad] Electrical angle.
 * @param  Id      [out] [A] D-axis current.
 * @param  Iq      [out] [A] Q-axis current.
 */
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id,
                    float *Iq);

/**
 * @brief  Inverse Park transform: dq→ αβ.
 * @param  Vd      [V] D-axis voltage.
 * @param  Vq      [V] Q-axis voltage.
 * @param  theta   [rad] Electrical angle.
 * @param  Valpha  [out] [V] Alpha-axis voltage.
 * @param  Vbeta   [out] [V] Beta-axis voltage.
 */
void Park_Inverse(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PARK_H */
