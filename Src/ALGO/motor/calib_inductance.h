#ifndef CALIB_INDUCTANCE_H
#define CALIB_INDUCTANCE_H

#include "calibration_context.h"
#include "motor.h"
#include "config.h"

/**
 * @file calib_inductance.h
 * @brief Inductance calibration module
 */

/**
 * @brief Update inductance calibration state machine
 *
 * @param motor Motor data
 * @param ctx   Inductance calibration context
 * @return CalibResult Calibration result
 */
CalibResult InductanceCalib_Update(MOTOR_DATA *motor,
                                   InductanceCalibContext *ctx, float dt);

#endif // CALIB_INDUCTANCE_H
