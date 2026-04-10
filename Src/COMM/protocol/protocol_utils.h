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
 * @file protocol_utils.h
 * @brief CAN 协议帧量化工具函数（Vector 和 MIT 协议共用）
 *
 * 提供浮点数 ↔ 固定精度整数的线性映射，以及大端字节序的缓冲区读写。
 * 两个协议原来各自实现一份，现统一到此模块。
 */

#ifndef PROTOCOL_UTILS_H
#define PROTOCOL_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 16-bit 整数线性映射到浮点数
 * @param x       原始值 [0, 65535]
 * @param min_val 映射最小值
 * @param max_val 映射最大值
 * @return 浮点数
 */
static inline float Proto_Uint16ToFloat(uint16_t x, float min_val, float max_val)
{
    return ((float)x / 65535.0f) * (max_val - min_val) + min_val;
}

/**
 * @brief 浮点数线性映射到 16-bit 整数（自动限幅）
 * @param x       浮点值
 * @param min_val 映射最小值
 * @param max_val 映射最大值
 * @return 整数 [0, 65535]
 */
static inline uint16_t Proto_FloatToUint16(float x, float min_val, float max_val)
{
    float span = max_val - min_val;
    float scaled = (x - min_val) / span;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 1.0f) scaled = 1.0f;
    return (uint16_t)(scaled * 65535.0f);
}

/**
 * @brief 12-bit 整数线性映射到浮点数（MIT Cheetah 协议专用）
 * @param x       原始值 [0, 4095]
 * @param min_val 映射最小值
 * @param max_val 映射最大值
 * @return 浮点数
 */
static inline float Proto_Uint12ToFloat(uint16_t x, float min_val, float max_val)
{
    return ((float)(x & 0x0FFFu) / 4095.0f) * (max_val - min_val) + min_val;
}

/**
 * @brief 浮点数线性映射到 12-bit 整数（MIT Cheetah 协议专用，自动限幅）
 * @param x       浮点值
 * @param min_val 映射最小值
 * @param max_val 映射最大值
 * @return 整数 [0, 4095]
 */
static inline uint16_t Proto_FloatToUint12(float x, float min_val, float max_val)
{
    float span = max_val - min_val;
    float scaled = (x - min_val) / span;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 1.0f) scaled = 1.0f;
    return (uint16_t)(scaled * 4095.0f);
}

/**
 * @brief 从缓冲区读取大端序 16-bit 无符号整数
 * @param buf 指向 2 字节缓冲区的指针
 * @return uint16_t
 */
static inline uint16_t Proto_BufToUint16(const uint8_t *buf)
{
    return (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

/**
 * @brief 将 16-bit 无符号整数以大端序写入缓冲区
 * @param val 要写入的值
 * @param buf 指向 2 字节缓冲区的指针
 */
static inline void Proto_Uint16ToBuf(uint16_t val, uint8_t *buf)
{
    buf[0] = (uint8_t)((val >> 8) & 0xFFu);
    buf[1] = (uint8_t)(val & 0xFFu);
}

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_UTILS_H */
