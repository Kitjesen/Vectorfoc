#ifndef CURRENT_CALIB_H
#define CURRENT_CALIB_H

#include "calibration_context.h"
#include "motor.h"

/**
 * @file current_calib.h
 * @brief Current offset calibration module
 *
 * Responsible for acquiring current sensor zero offsets
 */

/**
 * @brief Initialize current offset calibration
 *
 * @param motor Motor data structure
 * @param ctx Calibration context
 * @return CalibResult Calibration result status
 */
CalibResult CurrentCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Update current offset calibration (called every control cycle)
 *
 * @param motor Motor data structure
 * @param ctx Calibration context
 * @return CalibResult Calibration result status
 */
CalibResult CurrentCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Finish current offset calibration and apply offsets
 *
 * @param motor Motor data structure
 * @param ctx Calibration context
 * @return CalibResult Calibration result status
 */
CalibResult CurrentCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx);

#endif // CURRENT_CALIB_H
