/**
 * @file bsp_encoder_mt6816.c
 * @brief MT6816 磁编码器驱动适配器
 * @note 实现 HAL_Encoder 接口，适配 MT6816 硬件
 */

#include "hal_encoder.h"
#include "spi.h"
#include "gpio.h"
#include <math.h>

/* SPI 配置 */
#define ENCODER_SPI_HANDLE      hspi1
#define ENCODER_CS_GPIO_Port    GPIOA
#define ENCODER_CS_Pin          GPIO_PIN_4

/* MT6816 配置 */
#define MT6816_RESOLUTION       (1 << 14)   // 14-bit encoder
#define MT6816_COUNTS_PER_REV   16384.0f
#define MT6816_2PI              6.283185307179586f

/* 静态变量 */
static float g_position = 0.0f;         // 机械位置 [rad]
static float g_velocity = 0.0f;         // 机械速度 [rad/s]
static float g_offset = 0.0f;           // 零点偏移 [rad]
static uint16_t g_raw_angle = 0;        // 原始角度值
static uint16_t g_last_raw_angle = 0;   // 上次角度值
static int32_t g_turns = 0;             // 圈数
static uint32_t g_last_update_time = 0; // 上次更新时间 [us]

/* 内部函数声明 */
static void bsp_encoder_init(void);
static void bsp_encoder_update(void);
static float bsp_encoder_get_position(void);
static float bsp_encoder_get_velocity(void);
static float bsp_encoder_get_electrical_angle(uint8_t pole_pairs);
static float bsp_encoder_get_electrical_velocity(uint8_t pole_pairs);
static void bsp_encoder_set_offset(float offset);
static float bsp_encoder_get_offset(void);
static uint16_t mt6816_read_angle(void);
static uint32_t micros(void);

/* 编码器接口实现 */
static const HAL_Encoder_Interface_t g_encoder_interface = {
    .init = bsp_encoder_init,
    .update = bsp_encoder_update,
    .get_position = bsp_encoder_get_position,
    .get_velocity = bsp_encoder_get_velocity,
    .get_electrical_angle = bsp_encoder_get_electrical_angle,
    .get_electrical_velocity = bsp_encoder_get_electrical_velocity,
    .set_offset = bsp_encoder_set_offset,
    .get_offset = bsp_encoder_get_offset,
};

/**
 * @brief 初始化 MT6816 编码器
 */
static void bsp_encoder_init(void)
{
    /* SPI 已在 MX_SPI1_Init() 中初始化 */
    
    /* 配置 CS 引脚 */
    HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET);
    
    /* 读取初始角度 */
    HAL_Delay(10);
    g_raw_angle = mt6816_read_angle();
    g_last_raw_angle = g_raw_angle;
    g_position = ((float)g_raw_angle / MT6816_COUNTS_PER_REV) * MT6816_2PI;
    g_last_update_time = micros();
}

/**
 * @brief 更新编码器数据
 * 
 * 在定时器中断中调用，更新位置和速度
 */
static void bsp_encoder_update(void)
{
    /* 读取当前角度 */
    g_raw_angle = mt6816_read_angle();
    
    /* 检测圈数变化 */
    int16_t delta = (int16_t)g_raw_angle - (int16_t)g_last_raw_angle;
    
    if (delta > (MT6816_RESOLUTION / 2))
    {
        /* 反向跨越零点 */
        g_turns--;
    }
    else if (delta < -(MT6816_RESOLUTION / 2))
    {
        /* 正向跨越零点 */
        g_turns++;
    }
    
    /* 计算绝对位置（包含圈数）*/
    g_position = ((float)g_raw_angle / MT6816_COUNTS_PER_REV + g_turns) * MT6816_2PI - g_offset;
    
    /* 计算速度 */
    uint32_t current_time = micros();
    float dt = (current_time - g_last_update_time) * 1e-6f;  // [s]
    
    if (dt > 0.0f)
    {
        float delta_angle = ((float)delta / MT6816_COUNTS_PER_REV) * MT6816_2PI;
        g_velocity = delta_angle / dt;
        
        /* 速度低通滤波 */
        const float alpha = 0.2f;
        static float velocity_filtered = 0.0f;
        velocity_filtered = alpha * g_velocity + (1.0f - alpha) * velocity_filtered;
        g_velocity = velocity_filtered;
    }
    
    /* 更新上次值 */
    g_last_raw_angle = g_raw_angle;
    g_last_update_time = current_time;
}

/**
 * @brief 获取机械位置
 */
static float bsp_encoder_get_position(void)
{
    return g_position;
}

/**
 * @brief 获取机械速度
 */
static float bsp_encoder_get_velocity(void)
{
    return g_velocity;
}

/**
 * @brief 获取电角度
 */
static float bsp_encoder_get_electrical_angle(uint8_t pole_pairs)
{
    /* 计算电角度 */
    float elec_angle = fmodf(g_position * pole_pairs, MT6816_2PI);
    
    /* 归一化到 [0, 2π] */
    if (elec_angle < 0.0f)
        elec_angle += MT6816_2PI;
    
    return elec_angle;
}

/**
 * @brief 获取电角速度
 */
static float bsp_encoder_get_electrical_velocity(uint8_t pole_pairs)
{
    return g_velocity * pole_pairs;
}

/**
 * @brief 设置零点偏移
 */
static void bsp_encoder_set_offset(float offset)
{
    g_offset = offset;
}

/**
 * @brief 获取零点偏移
 */
static float bsp_encoder_get_offset(void)
{
    return g_offset;
}

/**
 * @brief 读取 MT6816 角度
 * 
 * @return 14-bit 角度值 [0~16383]
 */
static uint16_t mt6816_read_angle(void)
{
    uint8_t tx_data[2] = {0x03, 0x00};  // 读取角度命令
    uint8_t rx_data[2] = {0};
    
    /* 拉低 CS */
    HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_RESET);
    
    /* SPI 传输 */
    HAL_SPI_TransmitReceive(&ENCODER_SPI_HANDLE, tx_data, rx_data, 2, 10);
    
    /* 拉高 CS */
    HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET);
    
    /* 解析角度值（14-bit）*/
    uint16_t angle = ((uint16_t)rx_data[0] << 6) | (rx_data[1] >> 2);
    angle &= 0x3FFF;  // 保留低 14 位
    
    return angle;
}

/**
 * @brief 获取微秒时间戳
 * 
 * 使用 DWT 实现，需要在初始化时启用 DWT
 */
static uint32_t micros(void)
{
    /* 假设使用 DWT 获取时间戳 */
    /* 需要在 main.c 中调用 DWT_Init() */
    extern uint32_t DWT_GetMicros(void);
    return DWT_GetMicros();
}

/**
 * @brief 注册 MT6816 编码器接口
 * 
 * 在系统初始化时调用
 */
void BSP_Encoder_MT6816_Register(void)
{
    HAL_Encoder_Register(&g_encoder_interface);
}

/**
 * @brief 获取原始角度值（调试用）
 */
uint16_t BSP_Encoder_GetRawAngle(void)
{
    return g_raw_angle;
}

