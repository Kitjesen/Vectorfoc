/**
 * @file    trap_traj.h
 * @brief   Trapezoidal trajectory planner.
 * @details
 * - Context: Generates smooth position/velocity profiles with acceleration
 * limits.
 * - Units:   Position [units], Velocity [units/s], Accel [units/s^2].
 * @copyright Copyright 2021 codenocold (MIT/GPL compat).
 */

#ifndef ALGORITHM_TRAP_TRAJ_H
#define ALGORITHM_TRAP_TRAJ_H

#include "main.h"
#include <stdbool.h>

/**
 * @brief Trajectory state structure.
 */
typedef struct {
  float Y;              /**< Current position */
  float Yd;             /**< Current velocity */
  float Ydd;            /**< Current acceleration */
  float Tf_;            /**< Total trajectory duration */
  float t;              /**< Current time in trajectory */
  bool trajectory_done; /**< Flag: true if trajectory finished */
  /* Internal plan state */
  float Xi_;
  float Xf_;
  float Vi_;
  float Ar_;
  float Vr_;
  float Dr_;
  float Ta_;
  float Tv_;
  float Td_;
  float yAccel_;
} TrajTypeDef;

/**
 * @brief  Plan a new trapezoidal trajectory.
 * @param  traj  Trajectory instance.
 * @param  Xf    Target final position.
 * @param  Xi    Initial position.
 * @param  Vi    Initial velocity.
 * @param  Vmax  Maximum velocity limit (>0).
 * @param  Amax  Maximum acceleration limit (>0).
 * @param  Dmax  Maximum deceleration limit (>0).
 */
void TRAJ_plan(TrajTypeDef *traj, float Xf, float Xi, float Vi, float Vmax,
               float Amax, float Dmax);

/**
 * @brief  Evaluate trajectory at time t.
 * @param  traj  Trajectory instance.
 * @param  t     Time elapsed since start.
 */
void TRAJ_eval(TrajTypeDef *traj, float t);

#endif // ALGORITHM_TRAP_TRAJ_H
