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
 * @file hal_encoder.h
 * @brief encoder
 * @note position/speed/velocityget
 */
#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief encoder
 */
typedef struct {
  /**
   * @brief initencoder
   */
  void (*init)(void);
  /**
   * @brief updateencoder
   *
   * timerinterrupt，updatepositionspeed/velocity
   */
  void (*update)(void);
  /**
   * @brief getposition
   * @return position [rad],  [0, 2π]
   */
  float (*get_position)(void);
  /**
   * @brief getspeed/velocity
   * @return speed/velocity [rad/s]
   */
  float (*get_velocity)(void);
  /**
   * @brief getangle
   * @param pole_pairs pole pairs
   * @return angle [rad],  [0, 2π]
   */
  float (*get_electrical_angle)(uint8_t pole_pairs);
  /**
   * @brief getspeed/velocity
   * @param pole_pairs pole pairs
   * @return speed/velocity [rad/s]
   */
  float (*get_electrical_velocity)(uint8_t pole_pairs);
  /**
   * @brief setoffset
   * @param offset offset [rad]
   */
  void (*set_offset)(float offset);
  /**
   * @brief getoffset
   * @return offset [rad]
   */
  float (*get_offset)(void);
} HAL_Encoder_Interface_t;
/**
 * @brief encoder
 */
int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface);
int MHAL_Encoder_Init(void);
int MHAL_Encoder_Update(void);
float MHAL_Encoder_GetPosition(void);
float MHAL_Encoder_GetVelocity(void);
float MHAL_Encoder_GetElectricalAngle(uint8_t pole_pairs);
float MHAL_Encoder_GetElectricalVelocity(uint8_t pole_pairs);
int MHAL_Encoder_SetOffset(float offset);
float MHAL_Encoder_GetOffset(void);
#ifdef __cplusplus
}
#endif
#endif /* HAL_ENCODER_H */
