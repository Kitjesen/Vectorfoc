/**
 * @file hall_encoder.c
 * @brief 霍尔传感器位置/速度估算实现（X-STAR-S 开发板）
 *
 * 硬件：TIM3 Hall 传感器模式，PC6(HA)/PC7(HB)/PC8(HC)
 *
 * 霍尔扇区表（120° 安装，标准 6 步序列）：
 *   扇区  HA HB HC  state  电角度中心
 *     1    1  0  0   4     0°  (0 rad)
 *     2    1  1  0   6     60° (π/3)
 *     3    0  1  0   2     120°(2π/3)
 *     4    0  1  1   3     180°(π)
 *     5    0  0  1   1     240°(4π/3)
 *     6    1  0  1   5     300°(5π/3)
 *
 * 注意：若实际转向或相序不同，调整 s_sector_angle[] 或交换相线即可，
 *       无需修改本文件其余逻辑。
 */
#include "hall_encoder.h"
#include "board_config_xstar.h"
#include "hal_abstraction.h"  /* HAL_GetMicroseconds() */
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ==========================================================================
   常量
   ========================================================================== */
#define HALL_PI             3.14159265358979f
#define HALL_2PI            6.28318530717959f
#define HALL_SECTOR_ANGLE   (HALL_2PI / 6.0f)  /* 60° = π/3 rad */

/* 速度滤波：低通一阶 α=0.2（调高加快响应，调低更平滑） */
#define HALL_VEL_LPF_ALPHA  0.2f

/* 速度超时：超过此时间无跳变，判定为静止 [µs] */
#define HALL_TIMEOUT_US     100000UL    /* 100ms */

/* ==========================================================================
   霍尔状态 → 扇区映射表
   state = (HC<<2) | (HB<<1) | HA，有效值 1~6，0和7为无效
   ========================================================================== */
static const uint8_t s_state_to_sector[8] = {
    0,  /* 0b000 = 0: 无效 */
    5,  /* 0b001 = 1: 扇区5  (HA=1,HB=0,HC=0) */
    3,  /* 0b010 = 2: 扇区3  (HA=0,HB=1,HC=0) */
    4,  /* 0b011 = 3: 扇区4  (HA=1,HB=1,HC=0) -- 等等，3=011即HA=1,HB=1,HC=0 */
    1,  /* 0b100 = 4: 扇区1  (HA=0,HB=0,HC=1) */
    6,  /* 0b101 = 5: 扇区6  (HA=1,HB=0,HC=1) */
    2,  /* 0b110 = 6: 扇区2  (HA=0,HB=1,HC=1) */
    0,  /* 0b111 = 7: 无效 */
};

/*
 * state 编码说明：
 *   state bit0 = HA (PC6), bit1 = HB (PC7), bit2 = HC (PC8)
 *   state = (HC<<2)|(HB<<1)|HA
 *
 * 扇区中心电角度（相对于扇区1起始）：
 *   扇区1 → 30°  (π/6)
 *   扇区2 → 90°  (π/2)
 *   扇区3 → 150° (5π/6)
 *   扇区4 → 210° (7π/6)
 *   扇区5 → 270° (3π/2)
 *   扇区6 → 330° (11π/6)
 */
static const float s_sector_angle[7] = {
    0.0f,                           /* 0: 无效占位 */
    HALL_PI / 6.0f,                 /* 扇区1:  30° */
    HALL_PI / 2.0f,                 /* 扇区2:  90° */
    5.0f * HALL_PI / 6.0f,         /* 扇区3: 150° */
    7.0f * HALL_PI / 6.0f,         /* 扇区4: 210° */
    3.0f * HALL_PI / 2.0f,         /* 扇区5: 270° */
    11.0f * HALL_PI / 6.0f,        /* 扇区6: 330° */
};

/* ==========================================================================
   全局数据
   ========================================================================== */
Hall_Handle_t hall_data = {
    .hall_state        = 0,
    .sector            = 0,
    .elec_angle_rad    = 0.0f,
    .mec_angle_rad     = 0.0f,
    .offset_rad        = 0.0f,
    .velocity_rad_s    = 0.0f,
    .last_capture_us   = 0,
    .capture_period_us = 0.0f,
    .pole_pairs        = 1,
    .direction         = true,
    .calib_valid       = true,
    .signal_valid      = false,
};

/* ==========================================================================
   内部工具函数
   ========================================================================== */

static uint8_t Hall_ReadState(void) {
    uint8_t ha = (HAL_GPIO_ReadPin(HW_HALL_PORT, HW_HALL_HA_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t hb = (HAL_GPIO_ReadPin(HW_HALL_PORT, HW_HALL_HB_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t hc = (HAL_GPIO_ReadPin(HW_HALL_PORT, HW_HALL_HC_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    return (uint8_t)((hc << 2) | (hb << 1) | ha);
}

/* ==========================================================================
   公开接口实现
   ========================================================================== */

void Hall_Init(void) {
    float saved_offset = hall_data.offset_rad;
    uint8_t saved_pole_pairs = hall_data.pole_pairs == 0u ? 1u : hall_data.pole_pairs;
    bool saved_calib_valid = hall_data.calib_valid;

    memset(&hall_data, 0, sizeof(hall_data));
    hall_data.offset_rad = saved_offset;
    hall_data.pole_pairs = saved_pole_pairs;
    hall_data.calib_valid = saved_calib_valid;
    /* 读取初始状态 */
    hall_data.hall_state = Hall_ReadState();
    hall_data.sector     = s_state_to_sector[hall_data.hall_state];
    hall_data.signal_valid = (hall_data.sector >= 1u && hall_data.sector <= 6u);
    if (hall_data.sector >= 1 && hall_data.sector <= 6) {
        hall_data.elec_angle_rad = s_sector_angle[hall_data.sector] + hall_data.offset_rad;
    }
    hall_data.last_capture_us = HAL_GetMicroseconds();
    /* 启动 TIM3 Hall 传感器模式（CubeMX 已配置，此处仅启动捕获） */
    HAL_TIMEx_HallSensor_Start_IT(&HW_HALL_TIMER);
}

void Hall_UpdateFromISR(void) {
    uint32_t now_us   = HAL_GetMicroseconds();
    uint32_t elapsed  = now_us - hall_data.last_capture_us;
    uint8_t  new_state = Hall_ReadState();

    /* 无效状态（000 或 111）：忽略 */
    if (new_state == 0 || new_state == 7) {
        hall_data.signal_valid = false;
        return;
    }

    uint8_t new_sector = s_state_to_sector[new_state];
    hall_data.signal_valid = (new_sector >= 1u && new_sector <= 6u);

    /* 方向判断：正转扇区递增（1→2→3→4→5→6→1），反转递减 */
    if (hall_data.sector != 0) {
        uint8_t expected_next = (hall_data.sector % 6) + 1;  /* 1→2→...→6→1 */
        hall_data.direction   = (new_sector == expected_next);
    }

    /* 更新状态 */
    hall_data.hall_state = new_state;
    hall_data.sector     = new_sector;

    /* 电角度：扇区中心 + 零偏 */
    float raw_angle = s_sector_angle[new_sector] + hall_data.offset_rad;
    /* 归一化到 [0, 2π) */
    while (raw_angle >= HALL_2PI) raw_angle -= HALL_2PI;
    while (raw_angle <  0.0f)    raw_angle += HALL_2PI;
    hall_data.elec_angle_rad = raw_angle;

    /* 机械角度：电角度 / 极对数，累积（简化：仅跟踪电角度换算） */
    hall_data.mec_angle_rad = hall_data.elec_angle_rad / (float)hall_data.pole_pairs;

    /* 速度计算：机械角速度 = 电角步长 / 时间 / 极对数 */
    if (elapsed > 0 && elapsed < HALL_TIMEOUT_US) {
        float period_s    = (float)elapsed * 1e-6f;
        /* 每步电角度 = 60° = π/3 */
        float raw_vel     = HALL_SECTOR_ANGLE / period_s / (float)hall_data.pole_pairs;
        float signed_vel  = hall_data.direction ? raw_vel : -raw_vel;
        /* 低通滤波 */
        hall_data.velocity_rad_s = hall_data.velocity_rad_s
            + HALL_VEL_LPF_ALPHA * (signed_vel - hall_data.velocity_rad_s);
        hall_data.capture_period_us = (float)elapsed;
    }

    hall_data.last_capture_us = now_us;
}

void Hall_SetPolePairs(uint8_t pp) {
    if (pp > 0) {
        hall_data.pole_pairs = pp;
    }
}

/* ==========================================================================
   Motor_HAL_EncoderInterface_t 实现（对接 HAL 抽象层）
   ========================================================================== */

static void Hall_HAL_Update(void) {
    /* 速度超时检测：长时间无跳变则置零 */
    uint32_t elapsed = HAL_GetMicroseconds() - hall_data.last_capture_us;
    if (elapsed > HALL_TIMEOUT_US) {
        hall_data.velocity_rad_s = 0.0f;
    }
}

static void Hall_HAL_GetData(Motor_HAL_EncoderData_t *data) {
    data->angle_rad    = hall_data.mec_angle_rad;
    data->velocity_rad = hall_data.velocity_rad_s;
    data->elec_angle   = hall_data.elec_angle_rad;
    data->raw_value    = (int32_t)hall_data.hall_state;
}

static void Hall_HAL_SetOffset(float offset) {
    hall_data.offset_rad = offset;
}

const Motor_HAL_EncoderInterface_t g_hall_encoder_interface = {
    .update     = Hall_HAL_Update,
    .get_data   = Hall_HAL_GetData,
    .set_offset = Hall_HAL_SetOffset,
};
