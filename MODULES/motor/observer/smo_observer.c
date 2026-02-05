#include "smo_observer.h"
#include "common.h"
#include <math.h>

#define SMO_PLL_BW_HZ 200.0f
#define SMO_BLEND_VEL_THRESH_RAD_S 10.0f

void SMO_Observer_Init(SMO_Observer_t *smo) {
  if (!smo)
    return;
  smo->alpha = 0.0f;
  smo->beta = 0.0f;
  smo->est_bemf_alpha = 0.0f;
  smo->est_bemf_beta = 0.0f;
  smo->est_angle = 0.0f;
  smo->est_velocity = 0.0f;
  smo->pll_angle = 0.0f;
  smo->pll_velocity = 0.0f;
  smo->pll_kp = 0.0f;
  smo->pll_ki = 0.0f;
}

void SMO_Observer_Update(void *pMemory, MOTOR_DATA *motor) {
  SMO_Observer_t *smo = (SMO_Observer_t *)pMemory;
  if (!smo || !motor)
    return;

  // Parameters
  float dt = 1.0f / 20000.0f; // Approx 50us loop
  float Rs = motor->parameters.Rs;
  float Ls = motor->parameters.Ls;
  if (Ls < 1e-6f)
    Ls = 1e-6f;
  float inv_Ls = 1.0f / Ls;

  // 输入：algo_output 中的 αβ 电流与电压
  float i_alpha = motor->algo_output.Ialpha;
  float i_beta = motor->algo_output.Ibeta;
  float v_alpha = motor->algo_output.Valpha;
  float v_beta = motor->algo_output.Vbeta;

  // Gains (Alpha is Gain, Beta is Filter Coeff for simplicity here)
  float K_slide = smo->alpha; // Sliding gain
  float K_filt = smo->beta;   // Filter constant (0.0 - 1.0)

  // --- Alpha Axis ---
  float err_alpha = smo->est_i_alpha - i_alpha;
  float z_alpha = (err_alpha > 0.0f) ? K_slide : -K_slide;

  // Current Observer: di/dt = (v - R*i - e - z)/L
  // i[k+1] = i[k] + dt/L * (v - R*i - e - z)
  float di_alpha =
      (v_alpha - Rs * smo->est_i_alpha - smo->est_bemf_alpha - z_alpha) *
      inv_Ls;
  smo->est_i_alpha += di_alpha * dt;

  // BEMF Reconstruction (Low Pass Filter of Z)
  // E[k+1] = E[k] * (1-beta) + Z * beta
  smo->est_bemf_alpha += (z_alpha - smo->est_bemf_alpha) * K_filt;

  // --- Beta Axis ---
  float err_beta = smo->est_i_beta - i_beta;
  float z_beta = (err_beta > 0.0f) ? K_slide : -K_slide;

  float di_beta =
      (v_beta - Rs * smo->est_i_beta - smo->est_bemf_beta - z_beta) * inv_Ls;
  smo->est_i_beta += di_beta * dt;

  smo->est_bemf_beta += (z_beta - smo->est_bemf_beta) * K_filt;

  // --- Angle Calculation ---
  // Atan2(-E_alpha, E_beta) for standard PMSM BEMF lagging
  smo->est_angle = atan2f(-smo->est_bemf_alpha, smo->est_bemf_beta);

  // PLL-based angle/velocity estimation
  float pll_kp =
      (smo->pll_kp > 0.0f) ? smo->pll_kp : (2.0f * SMO_PLL_BW_HZ);
  float pll_ki =
      (smo->pll_ki > 0.0f) ? smo->pll_ki : (0.25f * pll_kp * pll_kp);
  float phase_err = wrap_pm_pi(smo->est_angle - smo->pll_angle);
  smo->pll_velocity += pll_ki * phase_err * dt;
  smo->pll_angle += (smo->pll_velocity + pll_kp * phase_err) * dt;
  smo->pll_angle = wrap_pm_pi(smo->pll_angle);

  float enc_vel = motor->feedback.velocity * M_2PI;
  float abs_enc_vel = fabsf(enc_vel);
  if (abs_enc_vel < SMO_BLEND_VEL_THRESH_RAD_S) {
    float w = (SMO_BLEND_VEL_THRESH_RAD_S <= 0.0f)
                  ? 1.0f
                  : (abs_enc_vel / SMO_BLEND_VEL_THRESH_RAD_S);
    w = CLAMP(w, 0.0f, 1.0f);
    smo->est_velocity = w * smo->pll_velocity + (1.0f - w) * enc_vel;

    float enc_angle = motor->feedback.phase_angle;
    float angle_err = wrap_pm_pi(enc_angle - smo->pll_angle);
    smo->est_angle = wrap_pm_pi(smo->pll_angle + (1.0f - w) * angle_err);
  } else {
    smo->est_angle = smo->pll_angle;
    smo->est_velocity = smo->pll_velocity;
  }

  // Update Motor Feedback
  motor->feedback.observer_angle = smo->est_angle;
  motor->feedback.observer_velocity = smo->est_velocity;
}
