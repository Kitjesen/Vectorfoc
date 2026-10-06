// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "sliding_mode_observer.h"
#include "foc_transforms.h"
#include <math.h>
#include <stddef.h>

#define SMO_PLL_BW_HZ 200.0f
#define SMO_BLEND_VEL_THRESH_RAD_S 10.0f

static float clamp(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

/* Keep the observer's historical wrap arithmetic so moving it behind the
 * numeric interface does not change the low bits of the PLL state. */
static float wrap_pm_pi_legacy(float angle) {
  float wrapped = fmodf(angle + MATH_PI, 2.0f * MATH_PI);
  if (wrapped < 0.0f) wrapped += 2.0f * MATH_PI;
  return wrapped - MATH_PI;
}

void SMO_Observer_AlgorithmInit(SMO_ObserverState_t *state) {
  if (state == NULL) return;
  state->alpha = 0.0f;
  state->beta = 0.0f;
  state->est_i_alpha = 0.0f;
  state->est_i_beta = 0.0f;
  state->est_bemf_alpha = 0.0f;
  state->est_bemf_beta = 0.0f;
  state->est_angle = 0.0f;
  state->est_velocity = 0.0f;
  state->pll_angle = 0.0f;
  state->pll_velocity = 0.0f;
  state->pll_kp = 0.0f;
  state->pll_ki = 0.0f;
}

void SMO_Observer_AlgorithmUpdate(SMO_ObserverState_t *state,
                                   const SMO_ObserverInput_t *input,
                                   SMO_ObserverOutput_t *output) {
  if (state == NULL || input == NULL || output == NULL) return;

  float dt = input->dt_s;
  if (dt <= 0.0f || dt > 0.01f || !isfinite(dt)) dt = 1.0f / 20000.0f;
  float ls = input->ls < 1e-6f ? 1e-6f : input->ls;
  float inv_ls = 1.0f / ls;
  float slide = state->alpha;
  float filter = clamp(state->beta, 0.0f, 1.0f);

  float err_alpha = state->est_i_alpha - input->i_alpha;
  float z_alpha = err_alpha > 0.0f ? slide : -slide;
  float di_alpha = (input->v_alpha - input->rs * state->est_i_alpha -
                    state->est_bemf_alpha - z_alpha) * inv_ls;
  state->est_i_alpha += di_alpha * dt;
  state->est_bemf_alpha += (z_alpha - state->est_bemf_alpha) * filter;

  float err_beta = state->est_i_beta - input->i_beta;
  float z_beta = err_beta > 0.0f ? slide : -slide;
  float di_beta = (input->v_beta - input->rs * state->est_i_beta -
                   state->est_bemf_beta - z_beta) * inv_ls;
  state->est_i_beta += di_beta * dt;
  state->est_bemf_beta += (z_beta - state->est_bemf_beta) * filter;

  state->est_angle = atan2f(-state->est_bemf_alpha, state->est_bemf_beta);
  float pll_kp = state->pll_kp > 0.0f ? state->pll_kp : 2.0f * SMO_PLL_BW_HZ;
  float pll_ki = state->pll_ki > 0.0f ? state->pll_ki : 0.25f * pll_kp * pll_kp;
  float phase_error = wrap_pm_pi_legacy(state->est_angle - state->pll_angle);
  state->pll_velocity += pll_ki * phase_error * dt;
  state->pll_angle += (state->pll_velocity + pll_kp * phase_error) * dt;
  state->pll_angle = wrap_pm_pi_legacy(state->pll_angle);

  float encoder_velocity_rad_s = input->encoder_velocity_turn_s * MATH_2PI;
  float abs_velocity = fabsf(encoder_velocity_rad_s);
  if (abs_velocity < SMO_BLEND_VEL_THRESH_RAD_S) {
    float weight = clamp(abs_velocity / SMO_BLEND_VEL_THRESH_RAD_S, 0.0f, 1.0f);
    state->est_velocity = weight * state->pll_velocity +
                          (1.0f - weight) * encoder_velocity_rad_s;
    float angle_error = wrap_pm_pi_legacy(input->encoder_angle_rad - state->pll_angle);
    state->est_angle = wrap_pm_pi_legacy(state->pll_angle +
                                         (1.0f - weight) * angle_error);
  } else {
    state->est_angle = state->pll_angle;
    state->est_velocity = state->pll_velocity;
  }
  output->angle_rad = state->est_angle;
  output->velocity_rad_s = state->est_velocity;
}
