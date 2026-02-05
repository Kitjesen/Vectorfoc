/**
 * @file hal_adc.h
 * @brief ADC 硬件抽象层接口
 * @note 提供统一的电流/电压采样接口
 */

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADC 接口结构体
 */
typedef struct {
    /**
     * @brief 初始化 ADC
     */
    void (*init)(void);
    
    /**
     * @brief 启动 ADC 转换
     */
    void (*start)(void);
    
    /**
     * @brief 停止 ADC 转换
     */
    void (*stop)(void);
    
    /**
     * @brief 获取三相电流
     * @param Ia a 相电流 [A] (输出)
     * @param Ib b 相电流 [A] (输出)
     * @param Ic c 相电流 [A] (输出)
     */
    void (*get_current)(float *Ia, float *Ib, float *Ic);
    
    /**
     * @brief 获取母线电压
     * @return 母线电压 [V]
     */
    float (*get_vbus)(void);
    
    /**
     * @brief 获取温度
     * @return 温度 [°C]
     */
    float (*get_temperature)(void);
    
    /**
     * @brief 校准电流零点
     */
    void (*calibrate_current)(void);
    
} HAL_ADC_Interface_t;

/**
 * @brief 注册 ADC 接口
 */
int HAL_ADC_Register(const HAL_ADC_Interface_t *interface);

/**
 * @brief 初始化 ADC
 */
int HAL_ADC_Init(void);

/**
 * @brief 启动 ADC 转换
 */
int HAL_ADC_Start(void);

/**
 * @brief 停止 ADC 转换
 */
int HAL_ADC_Stop(void);

/**
 * @brief 获取三相电流
 */
int HAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic);

/**
 * @brief 获取母线电压
 */
float HAL_ADC_GetVbus(void);

/**
 * @brief 获取温度
 */
float HAL_ADC_GetTemperature(void);

/**
 * @brief 校准电流零点
 */
int HAL_ADC_CalibrateCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H */

