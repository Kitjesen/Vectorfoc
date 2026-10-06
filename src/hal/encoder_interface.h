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
 * @file encoder_interface.h
 * @brief encoder
 * @note position/speed/velocityget
 */
#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  float angle_rad, velocity_rad, elec_angle;
  int32_t raw_value;
} EncoderData;
/* Update once per ADC ISR, then read a snapshot. Existing offset conventions
 * stay in the selected driver; this adapter does not change their units. */
int MHAL_Encoder_Update(void);
void MHAL_Encoder_GetData(EncoderData *data);
float MHAL_Encoder_GetPosition(void);
float MHAL_Encoder_GetVelocity(void);
int MHAL_Encoder_SetOffset(float offset);
#ifdef __cplusplus
}
#endif
#endif /* HAL_ENCODER_H */
