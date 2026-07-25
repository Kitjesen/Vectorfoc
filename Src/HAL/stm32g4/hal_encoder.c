// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file hal_encoder.c
 * @brief Encoder HAL wrapper
 */
#include "hal_encoder.h"

#include "motor_hal_api.h"
#include "platform.h"
#include "position_sensor.h"

#include <stddef.h>

static int MHAL_Encoder_ReadData(Motor_HAL_EncoderData_t *data) {
  PositionSensorSample_t sample;
  if (data == NULL ||
      PositionSensor_GetLastSample(&sample) != POSITION_SENSOR_STATUS_OK) {
    return -1;
  }
  data->position_rad = sample.position_rad;
  data->angle_rad = sample.mechanical_angle_rad;
  data->velocity_rad = sample.velocity_rad_s;
  data->elec_angle = sample.electrical_angle_rad;
  data->raw_value = sample.native_raw;
  return 0;
}

int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface) {
  (void)interface;
  /* Runtime driver injection was replaced by compile-time PositionSensor
   * selection.  Reject this legacy path instead of pretending to register an
   * interface that would never be called. */
  return -1;
}

int MHAL_Encoder_Init(void) {
  return PositionSensor_Init() == POSITION_SENSOR_STATUS_OK ? 0 : -1;
}

int MHAL_Encoder_Update(uint8_t pole_pairs) {
  PositionSensorSample_t sample;
  if (pole_pairs == 0U) {
    return -1;
  }
  return PositionSensor_UpdateAndRead(pole_pairs, &sample) ==
                 POSITION_SENSOR_STATUS_OK
             ? 0
             : -1;
}

float MHAL_Encoder_GetPosition(void) {
  Motor_HAL_EncoderData_t data = {0};
  return MHAL_Encoder_ReadData(&data) == 0 ? data.position_rad : 0.0f;
}

float MHAL_Encoder_GetVelocity(void) {
  Motor_HAL_EncoderData_t data = {0};
  return MHAL_Encoder_ReadData(&data) == 0 ? data.velocity_rad : 0.0f;
}

float MHAL_Encoder_GetElectricalAngle(uint8_t pole_pairs) {
  Motor_HAL_EncoderData_t data = {0};
  (void)pole_pairs;
  return MHAL_Encoder_ReadData(&data) == 0 ? data.elec_angle : 0.0f;
}

float MHAL_Encoder_GetElectricalVelocity(uint8_t pole_pairs) {
  Motor_HAL_EncoderData_t data = {0};
  return MHAL_Encoder_ReadData(&data) == 0 ? data.velocity_rad * pole_pairs
                                           : 0.0f;
}

int MHAL_Encoder_ZeroPosition(void) {
  PositionSensorStatus_t status;
  if (!PositionSensor_IsInitialized()) {
    return -1;
  }
  CRITICAL_SECTION_BEGIN();
  status = PositionSensor_ZeroMechanicalPosition();
  CRITICAL_SECTION_END();
  return status == POSITION_SENSOR_STATUS_OK ? 0 : -1;
}

int MHAL_Encoder_SetOffset(float offset) {
  return PositionSensor_SetElectricalOffset(offset) == POSITION_SENSOR_STATUS_OK
             ? 0
             : -1;
}

float MHAL_Encoder_GetOffset(void) {
  float offset = 0.0f;
  (void)PositionSensor_GetElectricalOffset(&offset);
  return offset;
}
