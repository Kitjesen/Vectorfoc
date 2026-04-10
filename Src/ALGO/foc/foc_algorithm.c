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

/**
 * @file    foc_algorithm.c
 * @brief   FOC（，）
 */
#include "foc_algorithm.h"
#include "clarke.h"
#include "math_common.h"
#include "park.h"
#include "svpwm.h"
#include "trigonometry.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#ifndef MATH_PI
#define MATH_PI 3.14159265358979323846f
#endif
/* [FIX] voltage () */
#define ONE_OVER_SQRT3 0.57735026918962576f /* 1/√3,  */
#define VOLTAGE_MARGIN 0.95f               /* 5%safety */
/*  */
/*  */
void FOC_Algorithm_InitState(FOC_AlgorithmState_t *state) {
  if (state == NULL)
    return;
  memset(state, 0, sizeof(FOC_AlgorithmState_t));
}
void FOC_Algorithm_CurrentLoop(const FOC_AlgorithmInput_t *input,
                               const FOC_AlgorithmConfig_t *config,
                               FOC_AlgorithmState_t *state,
                               FOC_AlgorithmOutput_t *output) {
  if (input == NULL || config == NULL || state == NULL || output == NULL) {
    return;
  }
  // disableoutput
  if (!input->enabled) {
    output->Ta = 0.0f;
    output->Tb = 0.0f;
    output->Tc = 0.0f;
    output->overmodulation = false;
    output->voltage_saturated = false;
    output->current_limited = false;
    return;
  }
  // Step 1: Clarke (abc → αβ)
  // phasecurrent (Ia, Ib, Ic) phasecurrent (Ialpha, Ibeta)
  // : Ialpha = Ia, Ibeta = (Ia + 2*Ib) / sqrt(3)
  Clarke_Transform(input->Ia, input->Ib, input->Ic, &output->Ialpha,
                   &output->Ibeta);
  // Step 2: Park (αβ → dq)
  // phasecurrent (Ialpha, Ibeta) phasecurrent (Id, Iq)
  // angle theta_elec
  Park_Transform(output->Ialpha, output->Ibeta, input->theta_elec, &output->Id,
                 &output->Iq);
  // Step 3: currentfeedbackfilter
  // feedbackcurrent Id/Iq filter，
  // alpha = 2*pi*fc*Ts / (1 + 2*pi*fc*Ts) (Tustin )
  float omega_c = 2.0f * MATH_PI * config->current_filter_fc;
  float dt = config->Ts_current;
  
  // [FIX] 添加 dt 有效性检查，避免除零和异常滤波
  if (dt <= 0.0f || dt > 0.01f) {
    dt = 0.00005f;  // 默认 20kHz
  }
  
  float alpha_filter = (omega_c * dt) / (1.0f + omega_c * dt);
  if (config->current_filter_fc <= 0.0f)
    alpha_filter = 1.0f; // frequency <= 0，filter（）
  state->Id_filt += alpha_filter * (output->Id - state->Id_filt);
  state->Iq_filt += alpha_filter * (output->Iq - state->Iq_filt);
  // Step 4: current
  // limitcurrent，
  //  sqrt(Id^2 + Iq^2) > limit，proportional Id  Iq
  float I_mag =
      sqrtf(input->Id_ref * input->Id_ref + input->Iq_ref * input->Iq_ref);
  float Id_cmd = input->Id_ref;
  float Iq_cmd = input->Iq_ref;
  if (I_mag > config->current_limit) {
    float scale = config->current_limit / I_mag;
    Id_cmd *= scale;
    Iq_cmd *= scale;
    output->current_limited = true;
  } else {
    output->current_limited = false;
  }
  // Step 5: PIcurrent (calcsaturation)
  // V_pi = Kp*err + I
  // I += (Ki*err + Kb*(V_sat - V_raw)) * Ts
  //  PI ，integralsaturation (Back-calculation)
  // Daxis (/)
  float error_d = Id_cmd - state->Id_filt; // filterfeedbackcalcerror
  float Vd_raw = config->Kp_current_d * error_d + state->integral_d;
  // Qaxis ()
  float error_q = Iq_cmd - state->Iq_filt;
  float Vq_raw = config->Kp_current_q * error_q + state->integral_q;
  // Step 6: feedforwarddecoupling
  //  D/Q axis，
  // Vd_ff = -omega_e * Ls * Iq
  // Vq_ff = omega_e * (Ls * Id + flux)
  float Vd_ff = 0.0f;
  float Vq_ff = 0.0f;
  if (config->enable_decoupling) {
    float omega_e = input->omega_elec;
    Vd_ff = -omega_e * config->Ls * state->Iq_filt;
    Vq_ff = omega_e * (config->Ls * state->Id_filt + config->flux);
    Vd_ff *= config->decoupling_gain;
    Vq_ff *= config->decoupling_gain;
  }
  // saturationvoltage (output + feedforward)
  output->Vd = Vd_raw + Vd_ff;
  output->Vq = Vq_raw + Vq_ff;
  // Step 7: voltage
  // limitvoltageinverteroutput
  // V_max = (Vbus / sqrt(3)) * 0.95f ( 5% safety)
  float V_max = input->Vbus * ONE_OVER_SQRT3 * VOLTAGE_MARGIN;
  float V_mag = sqrtf(output->Vd * output->Vd + output->Vq * output->Vq);
  if (V_mag > V_max) {
    float scale = V_max / V_mag;
    output->Vd *= scale;
    output->Vq *= scale;
    output->voltage_saturated = true;
  } else {
    output->voltage_saturated = false;
  }
  // integralsaturationupdate (Back-calculation anti-windup)
  // voltagesaturation，stopintegralintegral，integral
  // Kb (Back-calculation gain)  1/Kp
  float Kb = config->Kb_current;
  state->integral_d +=
      (config->Ki_current_d * error_d + Kb * (output->Vd - (Vd_raw + Vd_ff))) *
      dt;
  state->integral_q +=
      (config->Ki_current_q * error_q + Kb * (output->Vq - (Vq_raw + Vq_ff))) *
      dt;
  // Step 8: Park (dq → αβ)
  // voltage (Vd, Vq) phase (Valpha, Vbeta)
  Park_Inverse(output->Vd, output->Vq, input->theta_elec, &output->Valpha,
               &output->Vbeta);
  // Step 9: SVPWMmodulation (αβ → abc PWM)
  // modulationphaseduty cycle (Ta, Tb, Tc)
  // calc
  int ret = SVPWM_Modulate(output->Valpha, output->Vbeta, input->Vbus,
                           &output->Ta, &output->Tb, &output->Tc);
  output->overmodulation = (ret != 0);
}
void FOC_Algorithm_ResetState(FOC_AlgorithmState_t *state) {
  if (state == NULL)
    return;
  state->integral_d = 0.0f;
  state->integral_q = 0.0f;
  state->Id_filt = 0.0f;
  state->Iq_filt = 0.0f;
}
bool FOC_Algorithm_ValidateConfig(const FOC_AlgorithmConfig_t *config) {
  if (config == NULL)
    return false;
  // checkmotorparam
  if (config->Rs <= 0.0f || config->Rs > 100.0f)
    return false;
  if (config->Ls <= 0.0f || config->Ls > 1.0f)
    return false;
  if (config->flux < 0.0f || config->flux > 1.0f)
    return false;
  if (config->pole_pairs == 0 || config->pole_pairs > 50)
    return false;
  // checkPIparam
  if (config->Kp_current_d < 0.0f || config->Ki_current_d < 0.0f)
    return false;
  if (config->Kp_current_q < 0.0f || config->Ki_current_q < 0.0f)
    return false;
  // checksampleperiod
  if (config->Ts_current <= 0.0f || config->Ts_current > 0.01f)
    return false;
  // checklimit
  if (config->voltage_limit <= 0.0f)
    return false;
  if (config->current_limit <= 0.0f)
    return false;
  // checkgain
  if (config->decoupling_gain < 0.0f || config->decoupling_gain > 1.0f)
    return false;
  return true;
}
void FOC_Algorithm_CalculateCurrentGains(float Rs, float Ls, float bandwidth,
                                         float *Kp, float *Ki) {
  if (Kp == NULL || Ki == NULL)
    return;
  
  // [FIX] 添加参数有效性检查
  if (Ls <= 0.0f || Rs <= 0.0f || bandwidth <= 0.0f) {
    *Kp = 0.0f;
    *Ki = 0.0f;
    return;
  }
  
  // current ( IMC )
  float omega_c = 2.0f * MATH_PI * bandwidth;
  *Kp = Ls * omega_c;
  *Ki = Rs * omega_c;
}
/* Private function implementations */
