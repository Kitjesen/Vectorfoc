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
 * @file hall_encoder.h
 * @brief 霍尔传感器位置/速度估算（X-STAR-S 开发板）
 *
 * 使用 TIM3 Hall 传感器模式，PC6(HA)/PC7(HB)/PC8(HC)。
 * 分辨率：60° 电角度（6扇区）。
 * 速度：相邻两次霍尔跳变的时间间隔换算。
 */
#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include "motor_hal_api.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
   霍尔传感器数据结构
   ========================================================================== */
typedef struct {
    /* 当前状态 */
    uint8_t  hall_state;        /* 3bit霍尔状态 (HC<<2 | HB<<1 | HA) */
    uint8_t  sector;            /* 当前扇区 1~6 */

    /* 角度 */
    float    elec_angle_rad;    /* 电角度 [rad]，扇区中心值，范围 [0, 2π) */
    float    mec_angle_rad;     /* 机械角度（累积，多圈）[rad] */
    float    offset_rad;        /* 电角度零偏 [rad] */

    /* 速度 */
    float    velocity_rad_s;    /* 机械角速度 [rad/s] */
    uint32_t last_capture_us;   /* 上次跳变时刻 [µs] */
    float    capture_period_us; /* 两次跳变间隔 [µs] */

    /* 配置 */
    uint8_t  pole_pairs;        /* 极对数（调用方设置） */
    bool     direction;         /* 旋转方向：true=正转 */
    bool     calib_valid;       /* Hall 路径默认视为已校准 */
    bool     signal_valid;      /* 当前 Hall 状态是否有效 */
} Hall_Handle_t;

/* ==========================================================================
   公开接口
   ========================================================================== */

/**
 * @brief 初始化霍尔编码器，启动TIM3
 */
void Hall_Init(void);

/**
 * @brief 在TIM3霍尔捕获中断中调用，更新状态和速度
 * @note  对应 HAL_TIM_IC_CaptureCallback 或 TIM3_IRQHandler
 */
void Hall_UpdateFromISR(void);

/**
 * @brief 设置极对数（影响电角度和电速度换算）
 */
void Hall_SetPolePairs(uint8_t pp);

/* ==========================================================================
   全局数据（供 motor_hal_xstar.c 访问）
   ========================================================================== */
extern Hall_Handle_t hall_data;

/* ==========================================================================
   HAL Handle（在 motor_hal_xstar.c 中定义，此处声明）
   ========================================================================== */
extern Motor_HAL_Handle_t xstar_hal_handle;

#endif /* HALL_ENCODER_H */
