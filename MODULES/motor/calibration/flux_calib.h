#ifndef FLUX_CALIB_H
#define FLUX_CALIB_H

#include "calibration_context.h"
#include "motor.h"

/**
 * @file flux_calib.h
 * @brief Flux calibration module
 *
 * Measures motor flux using Back-EMF method
 */

/**
 * @brief Initialize flux calibration
 */
CalibResult FluxCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Update flux calibration
 */
CalibResult FluxCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Finish flux calibration
 */
CalibResult FluxCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx);

#endif // FLUX_CALIB_H
