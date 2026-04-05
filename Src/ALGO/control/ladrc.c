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
 * @file    ladrc.c
 * @brief   Linear Active Disturbance Rejection Control (LADRC)
 *
 * @details
 *    LADRC ，speed/velocity。
 *
 *   : ẏ = b0*u + f(y, w)
 *     y: output (speed/velocity)
 *     u: input (Iq)
 *     f:  ( + )
 *     b0: gain
 *
 *   LESO ():
 *     ż1 = z2 + β1*(y - z1) + b0*u
 *     ż2 = β2*(y - z1)
 *      z1 ≈ y, z2 ≈ f ()
 *
 *   LSEF:
 *     u0 = kp * (r - z1)        (proportional)
 *     u  = (u0 - z2) / b0       ()
 *
 *   param ():
 *     β1 = 2 * ωo
 *     β2 = ωo²
 *     kp = ωc
 *     (ωo: observer, ωc: )
 *
 *   : ωo = 3~5 × ωc
 */
#include "ladrc.h"
#include <math.h>
#include <stddef.h>
void LADRC_Init(LADRC_State_t *state, const LADRC_Config_t *config) {
  if (state == NULL || config == NULL)
    return;
  state->z1 = 0.0f;
  state->z2 = 0.0f;
  state->output = 0.0f;
  state->initialized = false;
  LADRC_UpdateGains(state, config);
}
void LADRC_UpdateGains(LADRC_State_t *state, const LADRC_Config_t *config) {
  if (state == NULL || config == NULL)
    return;
  // :  s² + β1*s + β2 = (s + ωo)²
  state->beta1 = 2.0f * config->omega_o;
  state->beta2 = config->omega_o * config->omega_o;
  // LSEF gain
  state->kp = config->omega_c;
}
float LADRC_Calc(LADRC_State_t *state, const LADRC_Config_t *config,
                 float vel_ref, float vel_fdb, float u_prev, float dt) {
  if (state == NULL || config == NULL || dt <= 1e-6f)
    return 0.0f;
  // ：observerstateinitfeedback
  if (!state->initialized) {
    state->z1 = vel_fdb;
    state->z2 = 0.0f;
    state->initialized = true;
  }
  // ── LESO update () ──
  // observererror
  float e_obs = vel_fdb - state->z1;
  // stateupdate
  // ż1 = z2 + β1*e + b0*u
  // ż2 = β2*e
  float z1_dot = state->z2 + state->beta1 * e_obs + config->b0 * u_prev;
  float z2_dot = state->beta2 * e_obs;
  state->z1 += z1_dot * dt;
  state->z2 += z2_dot * dt;
  // ── LSEF  ──
  // u0 = kp * (ref - z1)
  float u0 = state->kp * (vel_ref - state->z1);
  // : u = (u0 - z2) / b0
  float output;
  if (fabsf(config->b0) > 1e-6f) {
    output = (u0 - state->z2) / config->b0;
  } else {
    // b0 ，proportional
    output = u0;
  }
  // output
  if (output > config->max_output) {
    output = config->max_output;
  } else if (output < -config->max_output) {
    output = -config->max_output;
  }
  state->output = output;
  return output;
}
void LADRC_Reset(LADRC_State_t *state) {
  if (state == NULL)
    return;
  state->z1 = 0.0f;
  state->z2 = 0.0f;
  state->output = 0.0f;
  state->initialized = false;
}
