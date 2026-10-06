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

#include "encoder_interface.h"
#include "motor_configuration.h"
#include "board_configuration.h"
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#include "tmr3109_encoder.h"
extern TMR3109_Handle_t tmr3109_encoder_data;
#define SENSOR tmr3109_encoder_data
#define SENSOR_RAW raw_angle
#else
#include "mt6816_encoder.h"
extern MT6816_Handle_t encoder_data;
#define SENSOR encoder_data
#define SENSOR_RAW raw_angle
#endif
int MHAL_Encoder_Update(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  TMR3109_Update(&SENSOR, CURRENT_SAMPLE_PERIOD_S);
#else
  MT6816_Update(&SENSOR, CURRENT_SAMPLE_PERIOD_S);
#endif
  return 0;
}
void MHAL_Encoder_GetData(EncoderData *data) {
  data->angle_rad = SENSOR.mec_angle_rad;
  data->velocity_rad = SENSOR.velocity_rad_s;
  data->elec_angle = SENSOR.elec_angle_rad;
  data->raw_value = (int32_t)SENSOR.SENSOR_RAW;
}
float MHAL_Encoder_GetPosition(void) { return SENSOR.mec_angle_rad; }
float MHAL_Encoder_GetVelocity(void) { return SENSOR.velocity_rad_s; }
int MHAL_Encoder_SetOffset(float offset) {
  SENSOR.offset_rev = offset;
  return 0;
}
