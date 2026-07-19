// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "position_sensor_motor_hal.h"

#include "position_sensor.h"

#include <string.h>

static uint8_t s_pole_pairs;

static bool PositionSensorMotorHal_Update(void) {
  PositionSensorSample_t sample;
  return PositionSensor_UpdateAndRead(s_pole_pairs, &sample) ==
         POSITION_SENSOR_STATUS_OK;
}

static void
PositionSensorMotorHal_GetData(Motor_HAL_EncoderData_t *data) {
  PositionSensorSample_t sample;
  if (data == NULL) {
    return;
  }
  if (PositionSensor_GetLastSample(&sample) != POSITION_SENSOR_STATUS_OK) {
    memset(data, 0, sizeof(*data));
    return;
  }
  data->position_rad = sample.position_rad;
  data->angle_rad = sample.mechanical_angle_rad;
  data->velocity_rad = sample.velocity_rad_s;
  data->elec_angle = sample.electrical_angle_rad;
  data->raw_value = sample.native_raw;
}

static void PositionSensorMotorHal_SetPolePairs(uint8_t pole_pairs) {
  if (pole_pairs == 0u || pole_pairs == s_pole_pairs) {
    return;
  }
  s_pole_pairs = pole_pairs;
  if (PositionSensor_IsInitialized()) {
    (void)PositionSensor_SetPolePairs(pole_pairs);
  }
}

static void PositionSensorMotorHal_ZeroPosition(void) {
  (void)PositionSensor_ZeroMechanicalPosition();
}

static void PositionSensorMotorHal_SetOffset(float offset) {
  (void)PositionSensor_SetElectricalOffset(offset);
}

static float PositionSensorMotorHal_GetOffset(void) {
  float offset = 0.0f;
  (void)PositionSensor_GetElectricalOffset(&offset);
  return offset;
}

const Motor_HAL_EncoderInterface_t g_position_sensor_motor_hal_interface = {
    .update = PositionSensorMotorHal_Update,
    .get_data = PositionSensorMotorHal_GetData,
    .set_pole_pairs = PositionSensorMotorHal_SetPolePairs,
    .zero_position = PositionSensorMotorHal_ZeroPosition,
    .set_offset = PositionSensorMotorHal_SetOffset,
    .get_offset = PositionSensorMotorHal_GetOffset,
};
