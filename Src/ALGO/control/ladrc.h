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
 * @file    ladrc.h
 * @brief   Linear Active Disturbance Rejection Control (LADRC) for speed loop
 * @details
 *   LADRC ：
 *   1. LESO (Linear Extended State Observer) - stateobserver
 *   2. TD (Tracking Differentiator) - derivative（）
 *   3. LSEF (Linear State Error Feedback) - stateerrorfeedback
 *
 *   phase PID，LADRC :
 *   - ""
 *   -
 *   -
 *   - （param：observer ωo  ωc）
 *
 * @note    speed/velocity， PI
 */
#ifndef ALGO_LADRC_H
#define ALGO_LADRC_H
#include "common.h"
/**
 * @brief LADRC param
 */
typedef struct {
  float omega_o;    /**< observer [rad/s] (: 3~5 × omega_c) */
  float omega_c;    /**<  [rad/s] (: 50~200 for speed loop) */
  float b0;         /**< gain b0 (Iq → velocity gain)
                         b0 ≈ (3/2) * pole_pairs * flux / J
                         J  */
  float max_output; /**< output [A] (Iq) */
} LADRC_Config_t;
/**
 * @brief LADRC state
 */
typedef struct {
  /* LESO state */
  float z1;     /**< observerstate1: speed/velocity */
  float z2;     /**< observerstate2:  */
  /* output */
  float output; /**< output (Iq_ref) */
  /* config */
  float beta1;  /**< LESO gain1 = 2 * omega_o */
  float beta2;  /**< LESO gain2 = omega_o^2 */
  float kp;     /**< LSEF proportionalgain = omega_c */
  bool initialized;
} LADRC_State_t;
/**
 * @brief  init LADRC
 * @param  state  LADRC state
 * @param  config LADRC configparam
 */
void LADRC_Init(LADRC_State_t *state, const LADRC_Config_t *config);
/**
 * @brief  LADRC calc
 * @param  state     LADRC state
 * @param  config    LADRC config
 * @param  vel_ref   speed/velocity [turn/s]
 * @param  vel_fdb   speed/velocityfeedback [turn/s]
 * @param  u_prev    periodoutput (Iq) [A]
 * @param  dt        sampleperiod [s]
 * @return output (Iq_ref) [A]
 */
float LADRC_Calc(LADRC_State_t *state, const LADRC_Config_t *config,
                 float vel_ref, float vel_fdb, float u_prev, float dt);
/**
 * @brief   LADRC state
 * @param  state  LADRC state
 */
void LADRC_Reset(LADRC_State_t *state);
/**
 * @brief  update LADRC param (/gain)
 * @param  state  LADRC state
 * @param  config LADRC config
 */
void LADRC_UpdateGains(LADRC_State_t *state, const LADRC_Config_t *config);
#endif // ALGO_LADRC_H
