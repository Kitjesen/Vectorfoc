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

#include "motor.h"
#ifdef BOARD_XSTAR
#include "board_config_xstar.h"
#include "hall_encoder.h"
#include "abz_encoder.h"
#else
#include "mt6816_encoder.h"
#endif

static int MHAL_Encoder_ReadData(Motor_HAL_EncoderData_t *data) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->encoder == NULL ||
      motor_data.components.hal->encoder->get_data == NULL) {
    return -1;
  }
  motor_data.components.hal->encoder->get_data(data);
  return 0;
}

int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface) {
  (void)interface;
  return 0;
}

int MHAL_Encoder_Init(void) {
#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  Hall_Init();
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
  Abz_Init();
#else
  return -1;
#endif
#endif
  return 0;
}

int MHAL_Encoder_Update(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->encoder == NULL ||
      motor_data.components.hal->encoder->update == NULL) {
    return -1;
  }
#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  Hall_SetPolePairs((uint8_t)motor_data.parameters.pole_pairs);
#else
  Abz_SetPolePairs((uint8_t)motor_data.parameters.pole_pairs);
#endif
#else
  encoder_data.pole_pairs = (uint8_t)motor_data.parameters.pole_pairs;
#endif
  motor_data.components.hal->encoder->update();
  return 0;
}

float MHAL_Encoder_GetPosition(void) {
  Motor_HAL_EncoderData_t data = {0};
  return MHAL_Encoder_ReadData(&data) == 0 ? data.angle_rad : 0.0f;
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
  return MHAL_Encoder_ReadData(&data) == 0 ? data.velocity_rad * pole_pairs : 0.0f;
}

int MHAL_Encoder_SetOffset(float offset) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->encoder == NULL ||
      motor_data.components.hal->encoder->set_offset == NULL) {
    return -1;
  }
  motor_data.components.hal->encoder->set_offset(offset);
  return 0;
}

float MHAL_Encoder_GetOffset(void) {
#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  return hall_data.offset_rad;
#else
  return abz_data.offset_rad;
#endif
#else
  return encoder_data.offset_rev;
#endif
}
