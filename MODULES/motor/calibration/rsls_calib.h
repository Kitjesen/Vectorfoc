#ifndef RSLS_CALIB_H
#define RSLS_CALIB_H

#include "calibration_context.h"
#include "motor.h"


/**
 * @file rsls_calib.h
 * @brief Resistance, Inductance, Direction, Pole Pair, and Encoder Calibration Module
 *
 * This module coordinates the complete automatic motor parameter identification flow.
 */

/**
 * @brief Initialize Rs/Ls calibration
 */
CalibResult RSLSCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx);

/**
 * @brief Update Rs/Ls calibration state machine
 */
CalibResult RSLSCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx,
                             float dt);

/**
 * @brief Get current calibration progress percentage
 */
uint8_t RSLSCalib_GetProgress(CalibrationContext *ctx);

#endif // RSLS_CALIB_H
