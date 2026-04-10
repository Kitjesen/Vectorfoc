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
 *
 * 层次设计说明：
 *   HAL 层不应反向依赖 ALGO 层的 motor_data 全局变量。
 *   此文件通过 MHAL_Encoder_SetHandle() 缓存 Motor_HAL_Handle_t 指针，
 *   由 app_init.c 在初始化时注入，消除 HAL→ALGO 的反向依赖。
 */
#include "hal_encoder.h"
#include "motor_hal_api.h"   /* Motor_HAL_Handle_t, Motor_HAL_EncoderData_t */
#ifdef BOARD_XSTAR
#include "board_config_xstar.h"
#include "hall_encoder.h"
#include "abz_encoder.h"
#else
#include "mt6816_encoder.h"
#endif

/* 由 app_init.c 注入，消除对 motor_data 的直接引用 */
static const Motor_HAL_Handle_t *s_hal = NULL;

void MHAL_Encoder_SetHandle(const void *hal_handle)
{
    s_hal = (const Motor_HAL_Handle_t *)hal_handle;
}

static int read_data(Motor_HAL_EncoderData_t *data)
{
    if (s_hal == NULL || s_hal->encoder == NULL ||
        s_hal->encoder->get_data == NULL) {
        return -1;
    }
    s_hal->encoder->get_data(data);
    return 0;
}

int MHAL_Encoder_Register(const HAL_Encoder_Interface_t *interface)
{
    (void)interface;
    return 0;
}

int MHAL_Encoder_Init(void)
{
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

void MHAL_Encoder_UpdatePolePairs(uint8_t pole_pairs)
{
#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    Hall_SetPolePairs(pole_pairs);
#else
    Abz_SetPolePairs(pole_pairs);
#endif
#else
    if (s_hal && s_hal->encoder) {
        /* MT6816 / TMR3109：pole_pairs 通过 encoder_data 的 public 字段设置 */
        extern MT6816_Handle_t encoder_data;
        encoder_data.pole_pairs = pole_pairs;
    }
#endif
}

int MHAL_Encoder_Update(void)
{
    if (s_hal == NULL || s_hal->encoder == NULL ||
        s_hal->encoder->update == NULL) {
        return -1;
    }
    s_hal->encoder->update();
    return 0;
}

float MHAL_Encoder_GetPosition(void)
{
    Motor_HAL_EncoderData_t data = {0};
    return read_data(&data) == 0 ? data.angle_rad : 0.0f;
}

float MHAL_Encoder_GetVelocity(void)
{
    Motor_HAL_EncoderData_t data = {0};
    return read_data(&data) == 0 ? data.velocity_rad : 0.0f;
}

float MHAL_Encoder_GetElectricalAngle(uint8_t pole_pairs)
{
    Motor_HAL_EncoderData_t data = {0};
    (void)pole_pairs;
    return read_data(&data) == 0 ? data.elec_angle : 0.0f;
}

float MHAL_Encoder_GetElectricalVelocity(uint8_t pole_pairs)
{
    Motor_HAL_EncoderData_t data = {0};
    return read_data(&data) == 0 ? data.velocity_rad * (float)pole_pairs : 0.0f;
}

int MHAL_Encoder_SetOffset(float offset)
{
    if (s_hal == NULL || s_hal->encoder == NULL ||
        s_hal->encoder->set_offset == NULL) {
        return -1;
    }
    s_hal->encoder->set_offset(offset);
    return 0;
}

float MHAL_Encoder_GetOffset(void)
{
#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
    return hall_data.offset_rad;
#else
    return abz_data.offset_rad;
#endif
#else
    extern MT6816_Handle_t encoder_data;
    return encoder_data.offset_rev;
#endif
}
