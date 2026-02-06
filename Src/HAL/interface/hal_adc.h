/**
 * @file hal_adc.h
 * @brief ADC 纭欢鎶借薄灞傛帴鍙?
 * @note 鎻愪緵缁熶竴鐨勭數娴?鐢靛帇閲囨牱鎺ュ彛
 */

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADC 鎺ュ彛缁撴瀯浣?
 */
typedef struct {
    /**
     * @brief 鍒濆鍖?ADC
     */
    void (*init)(void);
    
    /**
     * @brief 鍚姩 ADC 杞崲
     */
    void (*start)(void);
    
    /**
     * @brief 鍋滄 ADC 杞崲
     */
    void (*stop)(void);
    
    /**
     * @brief 鑾峰彇涓夌浉鐢垫祦
     * @param Ia a 鐩哥數娴?[A] (杈撳嚭)
     * @param Ib b 鐩哥數娴?[A] (杈撳嚭)
     * @param Ic c 鐩哥數娴?[A] (杈撳嚭)
     */
    void (*get_current)(float *Ia, float *Ib, float *Ic);
    
    /**
     * @brief 鑾峰彇姣嶇嚎鐢靛帇
     * @return 姣嶇嚎鐢靛帇 [V]
     */
    float (*get_vbus)(void);
    
    /**
     * @brief 鑾峰彇娓╁害
     * @return 娓╁害 [掳C]
     */
    float (*get_temperature)(void);
    
    /**
     * @brief 鏍″噯鐢垫祦闆剁偣
     */
    void (*calibrate_current)(void);
    
} HAL_ADC_Interface_t;

/**
 * @brief 娉ㄥ唽 ADC 鎺ュ彛
 */
int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface);

/**
 * @brief 鍒濆鍖?ADC
 */
int MHAL_ADC_Init(void);

/**
 * @brief 鍚姩 ADC 杞崲
 */
int MHAL_ADC_Start(void);

/**
 * @brief 鍋滄 ADC 杞崲
 */
int MHAL_ADC_Stop(void);

/**
 * @brief 鑾峰彇涓夌浉鐢垫祦
 */
int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic);

/**
 * @brief 鑾峰彇姣嶇嚎鐢靛帇
 */
float MHAL_ADC_GetVbus(void);

/**
 * @brief 鑾峰彇娓╁害
 */
float MHAL_ADC_GetTemperature(void);

/**
 * @brief 鏍″噯鐢垫祦闆剁偣
 */
int MHAL_ADC_CalibrateCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H */

