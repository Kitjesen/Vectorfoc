/**
 * @file    trap_traj.c
 * @brief   Trapezoidal trajectory planner implementation.
 */

#include "trap_traj.h"
#include "common.h"
#include <math.h>

/**
 * @brief Return sign of value (-1.0 or 1.0).
 */
static float sign_hard(float val) { return (signbit(val)) ? -1.0f : 1.0f; }

void TRAJ_plan(TrajTypeDef *traj, float Xf, float Xi, float Vi, float Vmax,
               float Amax, float Dmax) {
  if (traj == NULL) {
    return;
  }

  float dX = Xf - Xi;
  float stop_dist = (Vi * Vi) / (2.0f * Dmax);
  float dXstop = copysignf(stop_dist, Vi);
  float s = sign_hard(dX - dXstop);

  traj->Ar_ = s * Amax;
  traj->Dr_ = -s * Dmax;
  traj->Vr_ = s * Vmax;

  // Double deceleration check (start speed > cruise speed)
  if ((s * Vi) > (s * traj->Vr_)) {
    traj->Ar_ = -s * Amax;
  }

  // Time to reach cruise speed
  traj->Ta_ = (traj->Vr_ - Vi) / traj->Ar_;
  traj->Td_ = -traj->Vr_ / traj->Dr_;

  // Min displacement to reach cruise speed
  float dXmin = 0.5f * traj->Ta_ * (traj->Vr_ + Vi) +
                0.5f * traj->Td_ * traj->Vr_;

  // Determine profile type
  if (s * dX < s * dXmin) {
    // Short move: Triangle profile (never reach Vmax)
    traj->Vr_ =
        s * sqrtf(fmaxf((traj->Dr_ * SQ(Vi) + 2 * traj->Ar_ * traj->Dr_ * dX) /
                            (traj->Dr_ - traj->Ar_),
                        0.0f));
    traj->Ta_ = fmaxf(0.0f, (traj->Vr_ - Vi) / traj->Ar_);
    traj->Td_ = fmaxf(0.0f, -traj->Vr_ / traj->Dr_);
    traj->Tv_ = 0.0f;
  } else {
    // Long move: Trapezoidal profile
    traj->Tv_ = (dX - dXmin) / traj->Vr_;
  }

  traj->Tf_ = traj->Ta_ + traj->Tv_ + traj->Td_;
  traj->Xi_ = Xi;
  traj->Xf_ = Xf;
  traj->Vi_ = Vi;
  traj->yAccel_ = Xi + Vi * traj->Ta_ + 0.5f * traj->Ar_ * SQ(traj->Ta_);
}

void TRAJ_eval(TrajTypeDef *traj, float t) {
  if (traj == NULL) {
    return;
  }

  if (t < 0.0f) {
    // Pre-start
    traj->Y = traj->Xi_;
    traj->Yd = traj->Vi_;
    traj->Ydd = 0.0f;
  } else if (t < traj->Ta_) {
    // Acceleration
    traj->Y = traj->Xi_ + traj->Vi_ * t + 0.5f * traj->Ar_ * SQ(t);
    traj->Yd = traj->Vi_ + traj->Ar_ * t;
    traj->Ydd = traj->Ar_;
  } else if (t < traj->Ta_ + traj->Tv_) {
    // Coasting (Constant Velocity)
    traj->Y = traj->yAccel_ + traj->Vr_ * (t - traj->Ta_);
    traj->Yd = traj->Vr_;
    traj->Ydd = 0.0f;
  } else if (t < traj->Tf_) {
    // Deceleration
    float td = t - traj->Tf_;
    traj->Y = traj->Xf_ + 0.5f * traj->Dr_ * SQ(td);
    traj->Yd = traj->Dr_ * td;
    traj->Ydd = traj->Dr_;
  } else {
    // Done
    traj->Y = traj->Xf_;
    traj->Yd = 0.0f;
    traj->Ydd = 0.0f;
  }

  traj->trajectory_done = (t >= traj->Tf_);
  traj->t = t;
}
