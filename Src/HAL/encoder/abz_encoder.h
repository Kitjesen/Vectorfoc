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

#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H

#include "motor_hal_api.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t cpr;
  uint8_t pole_pairs;
  bool direction;
  bool calib_valid;
  bool index_seen;
  bool last_z_state;
  int32_t raw_count;
  int32_t count_in_cpr;
  int32_t last_count;
  int32_t offset_counts;
  int64_t shadow_count;
  uint32_t last_update_us;
  float offset_rad;
  float mec_angle_rad;
  float elec_angle_rad;
  float velocity_rad_s;
  float vel_estimate_;
} Abz_Handle_t;

void Abz_Init(void);
void Abz_SetPolePairs(uint8_t pp);
void Abz_ResetCount(void);

extern Abz_Handle_t abz_data;
extern const Motor_HAL_EncoderInterface_t g_abz_encoder_interface;

#endif /* ABZ_ENCODER_H */
