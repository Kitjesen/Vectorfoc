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
 * @file    tmr3109_encoder.c
 * @brief   MDT TMR3109 磁编码器驱动实现
 *
 * 关键设计说明：
 *
 * 1. SPI 帧（32 clock，Mode 1 CPOL=0/CPHA=1）
 *    TX：[Op=011][Addr=0x00][5'b0][16'b0] = 0x60000000
 *    RX：MISO 从第 5 个 clock 起有效
 *        bits[27:5]  = 23-bit 角度（MSB first）
 *        bits[4:1]   = 4-bit CRC
 *        bit[0]      = 芯片内部错误标志（0=正常）
 *    实际接收到的 4 字节 rx[0..3]：
 *        rx[0] = MISO bits[27:20]（RX 第一字节，对应 MOSI bits[31:24] 期间）
 *        rx[1] = MISO bits[19:12]
 *        rx[2] = MISO bits[11:4]
 *        rx[3] = MISO bits[3:0]（低 4 bit，含 CRC[3:0]）
 *    组合 28-bit 有效数据：raw32 = (rx[0]<<20)|(rx[1]<<12)|(rx[2]<<4)|(rx[3]>>4)
 *    23-bit 角度：angle23 = raw32 >> 5   （即 bits[27:5]）
 *    4-bit CRC：  crc4    = (raw32 >> 1) & 0x0F
 *    Error bit：  err     = raw32 & 0x01
 *
 * 2. CRC4 校验
 *    多项式：x⁴+x³+x²+1（= 0b11101，即 0x1D，但反序操作使用 0xB 见下文）
 *    初始值：0b0011 = 0x3
 *    校验范围：{1'b0, angle23[22:0]} = 24 bit（高位先入）
 *    正确时 CRC 余式 = 发送方计算的 CRC（需重新计算并比对）
 *
 * 3. PLL 速度估计
 *    与 MT6816 驱动完全相同，仅 CPR 常量不同（8388608 vs 16384）。
 *    高分辨率使得 PLL 速度估计在低速时更平滑（量化噪声降低 512 倍）。
 *
 * 4. 多圈累加
 *    使用 64-bit shadow_count 防止长时间运行溢出。
 */

#ifndef BOARD_XSTAR   /* 仅 VectorFOC 板编译，X-STAR 不使用本驱动 */

#include "tmr3109_encoder.h"
#include "board_config.h"
#include "common.h"
#include "config.h"
#include "error_manager.h"
#include "error_types.h"
#include <math.h>
#include <string.h>

/* ============================================================================
   编译期保护：仅当板级配置选择了 TMR3109 时才编译本文件主体
   ============================================================================ */
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109

/* ============================================================================
   私有宏
   ============================================================================ */
#define SQ(x) ((x) * (x))

/* SPI 错误计数达到此值才上报，避免偶发噪声刷屏 */
#define TMR3109_ERR_REPORT_THRESHOLD 100u

/* ============================================================================
   全局实例
   ============================================================================ */
TMR3109_Handle_t tmr3109_encoder_data = {
    .hspi          = &HW_ENC_SPI,
    .cs_port       = HW_ENC_CS_PORT,
    .cs_pin        = HW_ENC_CS_PIN,
    .pole_pairs    = 7,
    .dir           = TMR3109_DIR_CW,
    .offset_rev    = 0.0f,
    .offset_counts = 0,
    .pll_bandwidth = TMR3109_PLL_BW_DEFAULT,
    .raw_angle     = 0,
    .angle         = 0,
    .spi_err_count = 0,
    .crc_err_count = 0,
    .chip_err_count= 0,
    .shadow_count  = 0,
    .count_in_cpr  = 0,
    .calib_valid   = false,
    .pos_estimate_counts_ = 0.0f,
    .vel_estimate_counts_ = 0.0f,
    .pos_cpr_counts_      = 0.0f,
    .interpolation_       = 0.0f,
    .pos_estimate_  = 0.0f,
    .vel_estimate_  = 0.0f,
    .pos_cpr_       = 0.0f,
    .phase_         = 0.0f,
    .mec_angle_rad  = 0.0f,
    .elec_angle_rad = 0.0f,
    .velocity_rad_s = 0.0f,
    .last_status    = TMR3109_OK,
};

/* ============================================================================
   私有函数声明
   ============================================================================ */
static inline void tmr3109_cs_low(TMR3109_Handle_t *enc);
static inline void tmr3109_cs_high(TMR3109_Handle_t *enc);
static bool        tmr3109_read_raw(TMR3109_Handle_t *enc);
static bool        tmr3109_crc4_check(uint32_t angle23, uint8_t crc_rx);
static float       tmr3109_normalize_angle(float angle);

/* ============================================================================
   公共函数实现
   ============================================================================ */

void TMR3109_Init(TMR3109_Handle_t *enc,
                  SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
{
    if (enc == NULL) return;

    enc->hspi     = hspi;
    enc->cs_port  = cs_port;
    enc->cs_pin   = cs_pin;
    enc->pole_pairs    = 7;
    enc->dir           = TMR3109_DIR_CW;
    enc->offset_rev    = 0.0f;
    enc->offset_counts = 0;
    enc->pll_bandwidth = TMR3109_PLL_BW_DEFAULT;

    enc->raw_angle     = 0;
    enc->angle         = 0;
    enc->spi_err_count = 0;
    enc->crc_err_count = 0;
    enc->chip_err_count= 0;
    enc->shadow_count  = 0;
    enc->count_in_cpr  = 0;
    enc->calib_valid   = false;

    memset(enc->offset_lut, 0, sizeof(enc->offset_lut));

    enc->pos_estimate_counts_ = 0.0f;
    enc->vel_estimate_counts_ = 0.0f;
    enc->pos_cpr_counts_      = 0.0f;
    enc->interpolation_       = 0.0f;
    enc->pos_estimate_  = 0.0f;
    enc->vel_estimate_  = 0.0f;
    enc->pos_cpr_       = 0.0f;
    enc->phase_         = 0.0f;
    enc->mec_angle_rad  = 0.0f;
    enc->elec_angle_rad = 0.0f;
    enc->velocity_rad_s = 0.0f;
    enc->last_status    = TMR3109_OK;
}

TMR3109_Status_t TMR3109_Update(TMR3109_Handle_t *enc, float dt)
{
    if (enc == NULL) return TMR3109_ERR_SPI;

    /* dt 有效性保护：异常值回退到 20 kHz 周期 */
    if (dt <= 0.0f || dt > 0.1f || !isfinite(dt)) {
        dt = 0.00005f;
    }

    TMR3109_ProcessAngle(enc, dt);

    /* 将 PLL 速度从 [rev/s] 换算到 [rad/s] */
    enc->velocity_rad_s = enc->vel_estimate_ * M_2PI;

    return enc->last_status;
}

void TMR3109_ResetCount(TMR3109_Handle_t *enc)
{
    if (enc == NULL) return;
    enc->shadow_count         = 0;
    enc->pos_estimate_counts_ = 0.0f;
    enc->pos_estimate_        = 0.0f;
}

/* ============================================================================
   核心处理函数
   ============================================================================ */

void TMR3109_ProcessAngle(TMR3109_Handle_t *enc, float dt)
{
    if (dt <= 0.0f || dt > 0.1f) return;

    /* ── PLL 参数（基于带宽） ──────────────────────────────────────────── */
    float pll_kp = 2.0f * enc->pll_bandwidth;
    float pll_ki = 0.25f * SQ(pll_kp);

    /* 速度接近零时的 snap 阈值（防止积分漂移） */
    float snap_threshold = 0.5f * dt * pll_ki;

    /* ── 步骤 1：SPI 读取原始角度 ───────────────────────────────────────── */
    if (!tmr3109_read_raw(enc)) {
        /* SPI/CRC/芯片错误：保持上一次估计，不更新状态 */
        return;
    }

    /* 方向修正 */
    if (enc->dir == TMR3109_DIR_CW) {
        enc->angle = enc->raw_angle;
    } else {
        enc->angle = (TMR3109_CPR - 1u - enc->raw_angle);
    }

    /* ── 步骤 2：可选软件 LUT 非线性补偿 ──────────────────────────────── */
    int32_t cnt;
    if (enc->calib_valid) {
        /*
         * TMR3109 LUT 粒度：8388608 / 128 = 65536 counts/格。
         * 每格跨度远大于 MT6816（128 counts/格），线性插值足够。
         */
        uint32_t idx1 = enc->angle >> 16;          /* /65536 */
        uint32_t idx2 = (idx1 + 1u) % TMR3109_LUT_SIZE;
        int32_t  off1 = enc->offset_lut[idx1];
        int32_t  off2 = enc->offset_lut[idx2];
        int32_t  rem  = (int32_t)(enc->angle - (idx1 << 16));
        /* 线性插值：单格 65536 counts */
        int32_t  off  = off1 + (int32_t)(((int64_t)(off2 - off1) * rem) >> 16);
        cnt = (int32_t)enc->angle - off;
    } else {
        cnt = (int32_t)enc->angle;
    }

    /* 边界归一化到 [0, CPR) */
    if (cnt >= (int32_t)TMR3109_CPR) {
        cnt -= (int32_t)TMR3109_CPR;
    } else if (cnt < 0) {
        cnt += (int32_t)TMR3109_CPR;
    }

    /* ── 步骤 3：增量计算（跨圈检测）──────────────────────────────────── */
    int32_t old_cnt     = enc->count_in_cpr;
    enc->count_in_cpr   = cnt;
    int32_t delta_enc   = cnt - old_cnt;

    /* 半圈阈值跨圈处理 */
    if (delta_enc >  (int32_t)(TMR3109_CPR / 2u)) {
        delta_enc -= (int32_t)TMR3109_CPR;
    } else if (delta_enc < -(int32_t)(TMR3109_CPR / 2u)) {
        delta_enc += (int32_t)TMR3109_CPR;
    }

    enc->shadow_count += (int64_t)delta_enc;

    /* ── 步骤 4：PLL（单圈域） ──────────────────────────────────────────── */
    enc->pos_cpr_counts_ += dt * enc->vel_estimate_counts_;

    float delta_pos = (float)(enc->count_in_cpr -
                               (int32_t)floorf(enc->pos_cpr_counts_));
    delta_pos = wrap_pm(delta_pos, (float)(TMR3109_CPR / 2u));

    enc->pos_cpr_counts_ += dt * pll_kp * delta_pos;
    enc->pos_cpr_counts_  = fmodf_pos(enc->pos_cpr_counts_, TMR3109_CPR_F);

    enc->vel_estimate_counts_ += dt * pll_ki * delta_pos;

    bool snap = false;
    if (ABS(enc->vel_estimate_counts_) < snap_threshold) {
        enc->vel_estimate_counts_ = 0.0f;
        snap = true;
    }

    /* ── 步骤 5：子计数插值 ─────────────────────────────────────────────── */
    if (snap) {
        enc->interpolation_ = 0.5f;
    } else if (delta_enc != 0) {
        enc->interpolation_ = (delta_enc > 0) ? 0.0f : 1.0f;
    } else {
        enc->interpolation_ += dt * enc->vel_estimate_counts_;
        if (enc->interpolation_ > 1.0f) enc->interpolation_ = 1.0f;
        if (enc->interpolation_ < 0.0f) enc->interpolation_ = 0.0f;
    }

    /* ── 步骤 6：输出计算 ──────────────────────────────────────────────── */
    enc->vel_estimate_ = enc->vel_estimate_counts_ / TMR3109_CPR_F;
    enc->pos_estimate_ = (double)enc->shadow_count / TMR3109_CPR_F;
    enc->pos_cpr_      = enc->pos_cpr_counts_      / TMR3109_CPR_F;

    /* ── 步骤 7：电角度与机械角度 ──────────────────────────────────────── */
    int32_t corrected = enc->count_in_cpr - enc->offset_counts;
    while (corrected < 0)                    corrected += (int32_t)TMR3109_CPR;
    while (corrected >= (int32_t)TMR3109_CPR) corrected -= (int32_t)TMR3109_CPR;

    float interp_enc = (float)corrected + enc->interpolation_;

    float elec_rad_per_enc = (float)enc->pole_pairs * M_2PI / TMR3109_CPR_F;

    enc->phase_         = wrap_pm_pi(elec_rad_per_enc * interp_enc);
    enc->elec_angle_rad = enc->phase_;
    enc->mec_angle_rad  = tmr3109_normalize_angle(interp_enc * (M_2PI / TMR3109_CPR_F));
}

/* ============================================================================
   私有函数实现
   ============================================================================ */

static inline void tmr3109_cs_low(TMR3109_Handle_t *enc)
{
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
}

static inline void tmr3109_cs_high(TMR3109_Handle_t *enc)
{
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief CRC4 校验
 *
 * 规格来自 TMR3109 数据手册 §9.1.8：
 *   校验范围：{1'b0, angle23[22:0]} 共 24 bit
 *   多项式：  x⁴ + x³ + x² + 1  →  0b11101 = 0x1D
 *   初始值：  4'b0011
 *   方向：    MSB 先入
 *
 * @param  angle23  23-bit 角度值
 * @param  crc_rx   从 MISO 接收到的 4-bit CRC
 * @return true 校验通过
 */
static bool tmr3109_crc4_check(uint32_t angle23, uint8_t crc_rx)
{
    /* 构造 24-bit 输入：最高位补 0，然后是 23-bit 角度 */
    uint32_t data = angle23 & 0x7FFFFFu; /* 确保只取 23 bit */

    uint8_t crc = 0x03u; /* 初始值 0b0011 */

    /* 从 bit23（补的 0）到 bit0，共 24 bit，MSB first */
    for (int i = 23; i >= 0; i--) {
        uint8_t bit = (uint8_t)((data >> i) & 0x01u);
        /* 与 CRC 最高位异或后移入 */
        if (((crc >> 3u) & 0x01u) ^ bit) {
            /* 多项式 0x1D 的低4位为 1101，移位后异或 */
            crc = (uint8_t)(((crc << 1u) ^ 0x0Du) & 0x0Fu);
        } else {
            crc = (uint8_t)((crc << 1u) & 0x0Fu);
        }
    }

    return (crc == (crc_rx & 0x0Fu));
}

/**
 * @brief 通过 SPI 读取 TMR3109 原始角度
 *
 * 协议细节见文件顶部注释。
 *
 * @return true  读取成功（SPI OK，CRC 通过，无芯片错误）
 *         false 任一检查失败（enc->last_status 已更新）
 */
static bool tmr3109_read_raw(TMR3109_Handle_t *enc)
{
    /* TX：Op_code=011，Addr=0x00，共 32 bit */
    uint8_t tx[4] = {
        TMR3109_TX_READ_ANGLE_B3,
        TMR3109_TX_READ_ANGLE_B2,
        TMR3109_TX_READ_ANGLE_B1,
        TMR3109_TX_READ_ANGLE_B0
    };
    uint8_t rx[4] = {0, 0, 0, 0};

    tmr3109_cs_low(enc);
    HAL_StatusTypeDef hal_ret =
        HAL_SPI_TransmitReceive(enc->hspi, tx, rx, 4, TMR3109_SPI_TIMEOUT_MS);
    tmr3109_cs_high(enc);

    /* ── 检查 1：SPI 通信 ──────────────────────────────────────────────── */
    if (hal_ret != HAL_OK) {
        if (enc->spi_err_count < 0xFFFFFFFFu) enc->spi_err_count++;
        if (enc->spi_err_count == 1u ||
            enc->spi_err_count % TMR3109_ERR_REPORT_THRESHOLD == 0u) {
            ERROR_REPORT(ERROR_MOTOR_ENCODER_SPI, "TMR3109 SPI error");
        }
        enc->last_status = TMR3109_ERR_SPI;
        return false;
    }
    if (enc->spi_err_count > 0u) enc->spi_err_count--;

    /*
     * ── 解析 MISO 28-bit 有效数据 ──────────────────────────────────────
     * TMR3109 MISO 输出从第 5 个 clock 开始，在 STM32 HAL 全双工模式下，
     * 芯片对 MOSI 的响应延后一个字节，实际 rx 布局：
     *
     *   rx[0] = MISO[27:20]  （芯片在第 1~8 clock 输出的高字节）
     *   rx[1] = MISO[19:12]
     *   rx[2] = MISO[11:4]
     *   rx[3] = MISO[3:0] 位于 bits[7:4]，bits[3:0] 为无效 Hi-Z
     *
     * 注：TMR3109 MISO 在 CS 拉低后延迟约 4 个 clock（即 Op_code 期间）
     * 才开始有效输出，因此接收字节整体偏移半字节。
     * 组合公式：raw28 = (rx[0]<<20)|(rx[1]<<12)|(rx[2]<<4)|(rx[3]>>4)
     */
    uint32_t raw28 = ((uint32_t)rx[0] << 20u) |
                     ((uint32_t)rx[1] << 12u) |
                     ((uint32_t)rx[2] <<  4u) |
                     ((uint32_t)rx[3] >>  4u);

    uint32_t angle23 = (raw28 >> 5u) & 0x7FFFFFu;  /* bits[27:5] */
    uint8_t  crc4    = (uint8_t)((raw28 >> 1u) & 0x0Fu); /* bits[4:1] */
    uint8_t  err_bit = (uint8_t)(raw28 & 0x01u);           /* bit[0]   */

    /* ── 检查 2：CRC4 ──────────────────────────────────────────────────── */
    if (!tmr3109_crc4_check(angle23, crc4)) {
        if (enc->crc_err_count < 0xFFFFFFFFu) enc->crc_err_count++;
        if (enc->crc_err_count == 1u ||
            enc->crc_err_count % TMR3109_ERR_REPORT_THRESHOLD == 0u) {
            ERROR_REPORT(ERROR_HW_ENCODER_SPI, "TMR3109 CRC4 error");
        }
        enc->last_status = TMR3109_ERR_CRC;
        return false;
    }
    if (enc->crc_err_count > 0u) enc->crc_err_count--;

    /* ── 检查 3：芯片内部错误标志 ──────────────────────────────────────── */
    if (err_bit != 0u) {
        if (enc->chip_err_count < 0xFFFFFFFFu) enc->chip_err_count++;
        if (enc->chip_err_count % TMR3109_ERR_REPORT_THRESHOLD == 0u) {
            ERROR_REPORT(ERROR_HW_ENCODER_LOSS, "TMR3109 internal error flag");
        }
        enc->last_status = TMR3109_ERR_CHIP;
        /* 芯片错误时角度仍可能有效，视应用场景决定是否继续；
         * 保守处理：丢弃本帧 */
        return false;
    }
    if (enc->chip_err_count > 0u) enc->chip_err_count--;

    enc->raw_angle   = angle23;
    enc->last_status = TMR3109_OK;
    return true;
}

/**
 * @brief 将角度归一化到 [0, 2π)
 */
static float tmr3109_normalize_angle(float angle)
{
    float a = fmodf(angle, M_2PI);
    return (a >= 0.0f) ? a : (a + M_2PI);
}

#endif /* HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109 */

#endif /* BOARD_XSTAR */
