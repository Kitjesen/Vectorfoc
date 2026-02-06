/**
 * @file    svpwm.h
 * @brief   Space Vector PWM (SVPWM) modulation.
 * @details Pure algorithm, no hardware dependency.
 */

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SVPWM modulation: αβ voltage → PWM duty cycles.
 * @param  Valpha  [V] Alpha-axis voltage reference.
 * @param  Vbeta   [V] Beta-axis voltage reference.
 * @param  Vbus    [V] Bus voltage.
 * @param  Ta      [out] [0~1] Phase A duty cycle.
 * @param  Tb      [out] [0~1] Phase B duty cycle.
 * @param  Tc      [out] [0~1] Phase C duty cycle.
 * @return 0 on success, -1 on over-modulation (voltage saturation).
 * @note   Uses midpoint injection method.
 */
int SVPWM_Modulate(float Valpha, float Vbeta, float Vbus, float *Ta, float *Tb,
                   float *Tc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVPWM_H */
