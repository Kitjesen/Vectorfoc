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
 * @file    tmr3109_encoder.h
 * @brief   MDT TMR3109 磁编码器驱动（23-bit，SPI Mode 1）
 *
 * @details
 * TMR3109 是多维科技（MDT）基于隧道磁阻（TMR）技术的 23-bit 绝对值磁编码器。
 * 与现有 MT6816（AMR，14-bit）相比：
 *   - 分辨率：23-bit（8 388 608 CPR），是 MT6816 的 512 倍
 *   - 精度：±0.05°（自校准后），重复精度 ±0.03°
 *   - 温漂：全温区（-40~125°C）±0.5°，优于 AMR
 *   - 片上自动校准：匀速旋转 3~5 s 即可完成，无需软件 LUT 采样
 *   - 封装：SOP8，引脚布局与 MT6816 兼容，硬件可直接替换
 *   - 价格：约 $1.67，与 MT6816 相当
 *
 * ── SPI 协议（CPOL=0，CPHA=1，即 Mode 1）──────────────────────────────────
 * 每次传输 32 个 clock（4 字节），CS 低电平期间有效。
 *
 * 读角度帧（Op_code = 3'b011）：
 *   TX MOSI（32 bit）：
 *     [31:29] Op_code = 0b011
 *     [28:21] Addr    = 0x00（角度寄存器）
 *     [20:16] 固定 0
 *     [15:0]  固定 0（写数据位，读角度时无意义）
 *
 *   RX MISO（28 bit 有效，从第 5 个 clock 开始输出）：
 *     [27:5]  23-bit 绝对角度（MSB first）
 *     [4:1]   4-bit CRC（多项式 x⁴+x³+x²+1，初始值 0b0011）
 *     [0]     Error 标志（0 = 正常）
 *
 * 角度转换：θ_deg = raw23 / 8 388 608 × 360°
 *            θ_rad = raw23 / 8 388 608 × 2π
 *
 * CRC 计算范围：1'b0 拼接 23-bit 角度，共 24 bit。
 * ──────────────────────────────────────────────────────────────────────────
 *
 * @note  驱动字段与 MT6816_Handle_t 保持对齐，可通过最小改动在
 *        motor_hal_g431.c / calib_encoder.c / motor_data.c 中切换。
 *
 * @version 1.0
 */

#ifndef TMR3109_ENCODER_H
#define TMR3109_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
   Constants
   ============================================================================ */

/** 23-bit 分辨率：2^23 counts/rev */
#define TMR3109_CPR      8388608u
#define TMR3109_CPR_F    8388608.0f

/** LUT 大小（与 MT6816 保持一致，片上校准后通常不需要） */
#define TMR3109_LUT_SIZE 128

/** SPI 超时 [ms] */
#define TMR3109_SPI_TIMEOUT_MS  10

/** 默认 PLL 带宽 [Hz]（与 MT6816 驱动相同） */
#define TMR3109_PLL_BW_DEFAULT  2000.0f

/* ============================================================================
   SPI 帧定义
   ============================================================================ */

/**
 * MOSI 第一字节：Op_code(3b) + Addr(8b) 的高5位
 * Op_code 011 = Read_angle，Addr = 0x00
 * 32-bit MOSI = [011 | 0000_0000 | 00000 | 0000_0000_0000_0000]
 *             = 0x6000_0000
 */
#define TMR3109_TX_READ_ANGLE_B3  0x60u   /* bits[31:24]: 0b0110_0000 */
#define TMR3109_TX_READ_ANGLE_B2  0x00u   /* bits[23:16] */
#define TMR3109_TX_READ_ANGLE_B1  0x00u   /* bits[15:8]  */
#define TMR3109_TX_READ_ANGLE_B0  0x00u   /* bits[7:0]   */

/** MISO bit[0]：片内错误标志（0 = 正常） */
#define TMR3109_ERR_FLAG_MASK     0x01u

/* ============================================================================
   枚举
   ============================================================================ */

/** 旋转方向 */
typedef enum {
    TMR3109_DIR_CW  =  1,
    TMR3109_DIR_CCW = -1
} TMR3109_Direction_t;

/** 驱动状态码 */
typedef enum {
    TMR3109_OK         = 0,  /**< 正常               */
    TMR3109_ERR_SPI    = 1,  /**< SPI 通信错误       */
    TMR3109_ERR_CRC    = 2,  /**< CRC4 校验失败      */
    TMR3109_ERR_CHIP   = 3,  /**< 芯片内部错误标志位 */
} TMR3109_Status_t;

/* ============================================================================
   句柄结构体
   ============================================================================ */

/**
 * @brief TMR3109 运行时句柄
 *
 * 字段命名与 MT6816_Handle_t 对齐，便于 motor_hal_g431.c 中最小改动切换。
 */
typedef struct {
    /* ── 硬件接口 ─────────────────────────────────────── */
    SPI_HandleTypeDef *hspi;    /**< SPI 句柄                     */
    GPIO_TypeDef      *cs_port; /**< CS GPIO 端口                 */
    uint16_t           cs_pin;  /**< CS GPIO 引脚掩码             */

    /* ── 配置 ────────────────────────────────────────── */
    uint8_t             pole_pairs;    /**< 电机极对数               */
    TMR3109_Direction_t dir;           /**< 旋转方向                 */
    float               offset_rev;   /**< 零位偏移 [rev]（0~1）    */
    int32_t             offset_counts; /**< 零位偏移 [counts]        */
    float               pll_bandwidth; /**< PLL 带宽 [Hz]            */

    /* ── 原始 / 处理后角度 ───────────────────────────── */
    uint32_t raw_angle;    /**< 最新 23-bit 读值（0~8388607）      */
    uint32_t angle;        /**< 方向修正后的角度                   */

    /* ── 错误计数 ─────────────────────────────────────── */
    uint32_t spi_err_count;  /**< SPI HAL 错误计数                */
    uint32_t crc_err_count;  /**< CRC4 校验失败计数               */
    uint32_t chip_err_count; /**< 芯片内部错误标志计数             */

    /* ── 多圈累加器 ──────────────────────────────────── */
    int64_t  shadow_count;   /**< 64-bit 多圈计数（防溢出）       */
    int32_t  count_in_cpr;   /**< 单圈计数 [0, CPR)               */

    /* ── 可选软件 LUT（片上校准后通常全零） ─────────── */
    int16_t offset_lut[TMR3109_LUT_SIZE]; /**< 非线性补偿 LUT       */
    bool    calib_valid;                  /**< 标定有效标志          */

    /* ── PLL 内部状态 ────────────────────────────────── */
    float pos_estimate_counts_;  /**< PLL 位置估计 [counts]        */
    float vel_estimate_counts_;  /**< PLL 速度估计 [counts/s]      */
    float pos_cpr_counts_;       /**< 单圈位置估计 [counts]        */
    float interpolation_;        /**< 子计数插值系数 [0, 1]        */

    /* ── PLL 输出（国际单位） ───────────────────────── */
    float pos_estimate_;   /**< 多圈位置估计 [rev]               */
    float vel_estimate_;   /**< 速度估计 [rev/s]                 */
    float pos_cpr_;        /**< 单圈位置 [rev]（0~1）            */
    float phase_;          /**< 电角度 (-π, π] [rad]             */

    /* ── 最终输出（供 motor_hal_g431 读取） ─────────── */
    float mec_angle_rad;   /**< 机械角度 [0, 2π) [rad]           */
    float elec_angle_rad;  /**< 电角度 (-π, π] [rad]             */
    float velocity_rad_s;  /**< 机械角速度 [rad/s]               */

    /* ── 状态 ────────────────────────────────────────── */
    TMR3109_Status_t last_status; /**< 最后一次读取状态            */

} TMR3109_Handle_t;

/* ============================================================================
   全局实例声明（motor_hal_g431.c 使用）
   ============================================================================ */
extern TMR3109_Handle_t tmr3109_encoder_data;

/* ============================================================================
   公共 API
   ============================================================================ */

/**
 * @brief  初始化 TMR3109 句柄。
 * @param  enc      句柄指针。
 * @param  hspi     已配置好的 SPI 句柄（Mode 1，CPOL=0，CPHA=1，≤10 MHz）。
 * @param  cs_port  CS GPIO 端口。
 * @param  cs_pin   CS GPIO 引脚掩码。
 */
void TMR3109_Init(TMR3109_Handle_t *enc,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin);

/**
 * @brief  更新编码器（控制环周期调用，如 20 kHz ISR）。
 * @param  enc  句柄指针。
 * @param  dt   控制周期 [s]。
 * @return 状态码。
 */
TMR3109_Status_t TMR3109_Update(TMR3109_Handle_t *enc, float dt);

/**
 * @brief  复位多圈计数器和 PLL 位置积分器。
 */
void TMR3109_ResetCount(TMR3109_Handle_t *enc);

/**
 * @brief  核心角度与速度处理（PLL、插值、SI 输出）。
 *         由 TMR3109_Update 内部调用，也可单独调用用于测试。
 */
void TMR3109_ProcessAngle(TMR3109_Handle_t *enc, float dt);

#ifdef __cplusplus
}
#endif

#endif /* TMR3109_ENCODER_H */
