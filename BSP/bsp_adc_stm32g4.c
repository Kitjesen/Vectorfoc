/**
 * @file bsp_adc_stm32g4.c
 * @brief STM32G4 ADC 驱动适配器
 * @note 实现 HAL_ADC 接口，适配 STM32G4 硬件
 */

#include "hal_adc.h"
#include "adc.h"
#include "main.h"
#include <math.h>

/* ADC 配置参数 */
#define ADC_HANDLE              hadc1
#define ADC_VREF                3.3f        // 参考电压 [V]
#define ADC_RESOLUTION          4096.0f     // 12-bit ADC

/* 电流采样参数 */
#define CURRENT_SENSE_GAIN      20.0f       // 电流采样放大倍数
#define CURRENT_SENSE_R         0.01f       // 采样电阻 [Ω]
#define CURRENT_SCALE           (ADC_VREF / ADC_RESOLUTION / CURRENT_SENSE_GAIN / CURRENT_SENSE_R)

/* 电压采样参数 */
#define VBUS_DIVIDER_RATIO      11.0f       // 电压分压比 (10K + 1K)
#define VBUS_SCALE              (ADC_VREF / ADC_RESOLUTION * VBUS_DIVIDER_RATIO)

/* NTC 温度查找表参数 */
#define NTC_B_VALUE             3950.0f
#define NTC_R0                  10000.0f
#define NTC_T0                  298.15f     // 25°C in Kelvin

/* 静态变量 */
static float g_current_offset_a = 0.0f;
static float g_current_offset_b = 0.0f;
static float g_current_offset_c = 0.0f;
static uint8_t g_calibrated = 0;

/* 内部函数声明 */
static void bsp_adc_init(void);
static void bsp_adc_start(void);
static void bsp_adc_stop(void);
static void bsp_adc_get_current(float *Ia, float *Ib, float *Ic);
static float bsp_adc_get_vbus(void);
static float bsp_adc_get_temperature(void);
static void bsp_adc_calibrate_current(void);

/* ADC 接口实现 */
static const HAL_ADC_Interface_t g_adc_interface = {
    .init = bsp_adc_init,
    .start = bsp_adc_start,
    .stop = bsp_adc_stop,
    .get_current = bsp_adc_get_current,
    .get_vbus = bsp_adc_get_vbus,
    .get_temperature = bsp_adc_get_temperature,
    .calibrate_current = bsp_adc_calibrate_current,
};

/**
 * @brief 初始化 STM32G4 ADC
 */
static void bsp_adc_init(void)
{
    /* STM32 HAL 已在 MX_ADC1_Init() 中初始化 */
    
    /* 校准 ADC */
    HAL_ADCEx_Calibration_Start(&ADC_HANDLE, ADC_SINGLE_ENDED);
    
    /* 校准电流零点 */
    bsp_adc_calibrate_current();
}

/**
 * @brief 启动 ADC 转换
 */
static void bsp_adc_start(void)
{
    /* 启动注入通道 ADC + 中断 */
    HAL_ADCEx_InjectedStart_IT(&ADC_HANDLE);
}

/**
 * @brief 停止 ADC 转换
 */
static void bsp_adc_stop(void)
{
    HAL_ADCEx_InjectedStop_IT(&ADC_HANDLE);
}

/**
 * @brief 获取三相电流
 */
static void bsp_adc_get_current(float *Ia, float *Ib, float *Ic)
{
    /* 读取 ADC 注入通道值 */
    uint32_t adc_a = HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_1);
    uint32_t adc_b = HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_2);
    uint32_t adc_c = HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_3);
    
    /* 转换为电流值 */
    *Ia = ((float)adc_a - g_current_offset_a) * CURRENT_SCALE;
    *Ib = ((float)adc_b - g_current_offset_b) * CURRENT_SCALE;
    *Ic = ((float)adc_c - g_current_offset_c) * CURRENT_SCALE;
}

/**
 * @brief 获取母线电压
 */
static float bsp_adc_get_vbus(void)
{
    /* 读取 ADC 规则通道值（假设在 Rank 4）*/
    uint32_t adc_vbus = HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_4);
    
    /* 转换为电压值 */
    float vbus = (float)adc_vbus * VBUS_SCALE;
    
    return vbus;
}

/**
 * @brief 获取温度
 * 
 * 使用 NTC 热敏电阻，通过 B 值公式计算温度
 */
static float bsp_adc_get_temperature(void)
{
    /* 读取 NTC ADC 值（假设通过普通通道）*/
    // 这里需要根据实际硬件配置修改
    // uint32_t adc_ntc = HAL_ADC_GetValue(&ADC_HANDLE);
    
    /* 示例：假设 ADC 值 */
    uint32_t adc_ntc = 2048;  // 中间值
    
    /* 计算 NTC 电阻 */
    float V_ntc = ((float)adc_ntc / ADC_RESOLUTION) * ADC_VREF;
    float R_ntc = NTC_R0 * V_ntc / (ADC_VREF - V_ntc);
    
    /* 使用 B 值公式计算温度 */
    float T_kelvin = 1.0f / (1.0f / NTC_T0 + logf(R_ntc / NTC_R0) / NTC_B_VALUE);
    float T_celsius = T_kelvin - 273.15f;
    
    return T_celsius;
}

/**
 * @brief 校准电流零点
 * 
 * 在电机静止时采样多次，计算零点偏移
 */
static void bsp_adc_calibrate_current(void)
{
    const int SAMPLES = 1000;
    float sum_a = 0.0f, sum_b = 0.0f, sum_c = 0.0f;
    
    /* 启动 ADC */
    HAL_ADCEx_InjectedStart(&ADC_HANDLE);
    
    /* 采样多次 */
    for (int i = 0; i < SAMPLES; i++)
    {
        /* 等待转换完成 */
        HAL_ADCEx_InjectedPollForConversion(&ADC_HANDLE, 10);
        
        /* 读取 ADC 值 */
        sum_a += (float)HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_1);
        sum_b += (float)HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_2);
        sum_c += (float)HAL_ADCEx_InjectedGetValue(&ADC_HANDLE, ADC_INJECTED_RANK_3);
        
        /* 延时 */
        HAL_Delay(1);
    }
    
    /* 停止 ADC */
    HAL_ADCEx_InjectedStop(&ADC_HANDLE);
    
    /* 计算平均值 */
    g_current_offset_a = sum_a / SAMPLES;
    g_current_offset_b = sum_b / SAMPLES;
    g_current_offset_c = sum_c / SAMPLES;
    
    g_calibrated = 1;
}

/**
 * @brief 注册 STM32G4 ADC 接口
 * 
 * 在系统初始化时调用
 */
void BSP_ADC_STM32G4_Register(void)
{
    HAL_ADC_Register(&g_adc_interface);
}

/**
 * @brief 检查 ADC 是否已校准
 */
uint8_t BSP_ADC_IsCalibrated(void)
{
    return g_calibrated;
}

