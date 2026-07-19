// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef POSITION_SENSOR_MOTOR_HAL_H
#define POSITION_SENSOR_MOTOR_HAL_H

#include "motor_hal_api.h"

/** Compatibility port for existing Motor_HAL_Handle_t consumers. */
extern const Motor_HAL_EncoderInterface_t g_position_sensor_motor_hal_interface;

#endif /* POSITION_SENSOR_MOTOR_HAL_H */
