#ifndef CALIB_RESISTANCE_H
#define CALIB_RESISTANCE_H

#include "calibration_context.h"
#include "motor.h"
#include "motor/config.h"

/**
 * @file calib_resistance.h
 * @brief Resistance calibration module
 */

/**
 * @brief Update resistance calibration state machine
 *
 * @param motor Motor data
 * @param ctx   Resistance calibration context
 * @return CalibResult Calibration result
 */
CalibResult ResistanceCalib_Update(MOTOR_DATA *motor,
                                   ResistanceCalibContext *ctx, float dt);

#endif // CALIB_RESISTANCE_H
