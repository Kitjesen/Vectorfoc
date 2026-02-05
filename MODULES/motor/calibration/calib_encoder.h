#ifndef CALIB_ENCODER_H
#define CALIB_ENCODER_H

#include "calibration_context.h"
#include "motor.h"
#include "motor/config.h"

/**
 * @file calib_encoder.h
 * @brief Encoder, pole pair, and direction calibration module
 */

/**
 * @brief Update direction and pole pair identification
 */
CalibResult DirectionPoleCalib_Update(MOTOR_DATA *motor,
                                      DirectionPoleCalibContext *ctx);

/**
 * @brief Update encoder offset calibration
 */
CalibResult EncoderCalib_Update(MOTOR_DATA *motor, EncoderCalibContext *ctx);

#endif // CALIB_ENCODER_H
