/**
 * @file foc_algorithm.h
 * @brief FOC ()
 *
 * @version 1.0
 * @date 2026-01-21
 * @author VectorFOC Team
 *
 * @note :
 *   - ，
 *   -
 *   -
 *   -
 *   -
 */
#ifndef FOC_ALGORITHM_H
#define FOC_ALGORITHM_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* ============================================================================
 *
 * ============================================================================
 */
typedef struct {
  /* === feedback === */
  float Ia;         ///< Aphasecurrent [A]
  float Ib;         ///< Bphasecurrent [A]
  float Ic;         ///< Cphasecurrent [A]
  float Vbus;       ///< voltage [V]
  float theta_elec; ///< angle [rad],  [-PI, PI]
  float omega_elec; ///< speed/velocity [rad/s]
  /* === reference === */
  float Id_ref; ///< Daxiscurrentreference [A]
  float Iq_ref; ///< Qaxiscurrentreference [A]
  /* === enable === */
  bool enabled; ///< enable
} FOC_AlgorithmInput_t;
/**
 * @brief FOCoutput
 *
 * FOCcalcoutput。
 * （PWMupdate）。
 */
typedef struct {
  /* === PWMduty cycleoutput === */
  float Ta; ///< Aphaseduty cycle [0~1]
  float Tb; ///< Bphaseduty cycle [0~1]
  float Tc; ///< Cphaseduty cycle [0~1]
  /* === （/） === */
  float Ialpha; ///< Alphaaxiscurrent [A]
  float Ibeta;  ///< Betaaxiscurrent [A]
  float Id;     ///< Daxiscurrent [A]
  float Iq;     ///< Qaxiscurrent [A]
  float Vd;     ///< Daxisvoltage [V]
  float Vq;     ///< Qaxisvoltage [V]
  float Valpha; ///< Alphaaxisvoltage [V]
  float Vbeta;  ///< Betaaxisvoltage [V]
  /* === state === */
  bool overmodulation;    ///< modulation
  bool voltage_saturated; ///< voltagesaturation
  bool current_limited;   ///< currentlimit
} FOC_AlgorithmOutput_t;
/**
 * @brief FOCconfigparam
 *
 * runningconfigparam。
 * initsetrunning。
 */
typedef struct {
  /* === motorparam === */
  float Rs;           ///< phase resistance [Ohm]
  float Ls;           ///< phase inductance [H]
  float flux;         ///< flux [Wb]
  uint8_t pole_pairs; ///< pole pairs
  /* === DaxiscurrentPIparam === */
  float Kp_current_d; ///< Daxisproportionalgain
  float Ki_current_d; ///< Daxisintegralgain
  /* === QaxiscurrentPIparam === */
  float Kp_current_q; ///< Qaxisproportionalgain
  float Ki_current_q; ///< Qaxisintegralgain
  /* === period === */
  float Ts_current; ///< currentsampleperiod [s]
  /* === limitparam === */
  float voltage_limit; ///< voltagelimit [V]
  float current_limit; ///< currentlimit [A]
  /* ===  === */
  bool enable_decoupling; ///< enableDQdecouplingfeedforward
  float decoupling_gain;  ///< decouplinggain (0.0  1.0)
  /* === currentparam === */
  float Kb_current;        ///< integralsaturationgain [1/s] ( Ki/Kp)
  float current_filter_fc; ///< currentfilterfrequency [Hz]
  
  /* === 死区补偿参数 === */
  float deadtime_i_thresh; ///< 电流过零插值阈值 [A]，建议 ADC 噪声的 2~3 倍
  float deadtime_Vdiode;   ///< 体二极管压降 [V]，典型 0.6~0.7V
} FOC_AlgorithmConfig_t;
typedef struct {
  /* === PIintegral === */
  float integral_d; ///< Daxiscurrentintegral
  float integral_q; ///< Qaxiscurrentintegral
  /* === filterstate（） === */
  float Id_filt; ///< filterDaxiscurrent [A]
  float Iq_filt; ///< filterQaxiscurrent [A]
} FOC_AlgorithmState_t;
/* ============================================================================
 *
 * ============================================================================
 */
void FOC_Algorithm_InitState(FOC_AlgorithmState_t *state);
/**
 * @brief FOCcalc
 * calc:
 *   1. Clarke: abc -> alpha-beta
 *   2. Park: alpha-beta -> dq
 *   3. PIcurrent: (Id_ref, Iq_ref) -> (Vd, Vq)
 *   4. feedforwarddecoupling ()
 *   5. voltagelimitintegralsaturation
 *   6. Park: dq -> alpha-beta
 *   7. SVPWMmodulation: alpha-beta -> abc PWM
 *
 * @param input    input（feedback + ）
 * @param config   config（）
 * @param state    state（）
 * @param output   output（calc）
 */
void FOC_Algorithm_CurrentLoop(const FOC_AlgorithmInput_t *input,
                               const FOC_AlgorithmConfig_t *config,
                               FOC_AlgorithmState_t *state,
                               FOC_AlgorithmOutput_t *output);
/**
 * @brief FOCstate
 *
 * integralfilterstate。
 * ：
 * - enablestate
 * - mode
 * - fault
 *
 * @param state state
 */
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state);
/**
 * @brief configparam
 *
 * checkconfigparam。
 * ：
 * - motorparam (Rs, Ls, flux, pole_pairs)
 * - PIgain (Kp, Ki)
 * - sampleperiod (Ts)
 * - limit (voltage_limit, current_limit)
 *
 * @param config config
 * @return true: , false:
 */
bool FOC_Algorithm_ValidateConfig(const FOC_AlgorithmConfig_t *config);
/* ============================================================================
 * （）
 * ============================================================================
 */
/**
 * @brief calcDQcurrentPIgain
 *
 * motorparamcalcPIgain。
 *  (IMC) 。
 *
 * @param Rs        phase resistance [Ohm]
 * @param Ls        phase inductance [H]
 * @param bandwidth  [Hz]
 * @param Kp        outputproportionalgain
 * @param Ki        outputintegralgain
 */
void FOC_Algorithm_CalculateCurrentGains(float Rs, float Ls, float bandwidth,
                                         float *Kp, float *Ki);
#ifdef __cplusplus
}
#endif
#endif /* FOC_ALGORITHM_H */
