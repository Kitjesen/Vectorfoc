/**
 * @file hal_adc.c
 * @brief ADC 纭欢鎶借薄灞傚疄鐜?
 * @note 缁熶竴 HAL 鏋舵瀯锛氬鎵樼粰 motor_data.components.hal->adc
 *       motor_data 鏄潤鎬佸垵濮嬪寲鐨勶紝绋嬪簭鍚姩鏃跺嵆鏈夋晥
 *       娉ㄦ剰锛氬嚱鏁板悕涓?header 淇濇寔涓€鑷?(HAL_ADC_*)
 */

#include "hal_adc.h"
#include "motor.h"

/* 缂撳瓨鏈€鏂扮殑 ADC 鏁版嵁 */
static Motor_HAL_SensorData_t s_adc_cache = {0};

/**
 * @brief 娉ㄥ唽 ADC 鎺ュ彛 (宸插簾寮?
 * @note 缁熶竴 HAL 鍚庝笉鍐嶉渶瑕佹敞鍐岋紝淇濈暀鍑芥暟绛惧悕浠ュ吋瀹归仐鐣欎唬鐮?
 */
int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface) {
  (void)interface; // 蹇界暐鍙傛暟锛屼笉鍐嶄娇鐢?
  return 0;        // 鎬绘槸杩斿洖鎴愬姛
}

int MHAL_ADC_Init(void) {
  // ADC 鍒濆鍖栫敱 MX_ADC1_Init/MX_ADC2_Init 鍦?main.c 涓畬鎴?
  return 0;
}

int MHAL_ADC_Start(void) {
  // ADC 鍚姩鐢?adc_bsp_init() 瀹屾垚
  return 0;
}

int MHAL_ADC_Stop(void) {
  // 涓嶅父鐢紝鍙暀绌?
  return 0;
}

int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->update == NULL)
    return -1;

  // 鏇存柊 ADC 鏁版嵁
  motor_data.components.hal->adc->update(&s_adc_cache);

  if (Ia)
    *Ia = s_adc_cache.i_a;
  if (Ib)
    *Ib = s_adc_cache.i_b;
  if (Ic)
    *Ic = s_adc_cache.i_c;

  return 0;
}

float MHAL_ADC_GetVbus(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->update == NULL)
    return 0.0f;

  motor_data.components.hal->adc->update(&s_adc_cache);
  return s_adc_cache.v_bus;
}

float MHAL_ADC_GetTemperature(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->update == NULL)
    return 0.0f;

  motor_data.components.hal->adc->update(&s_adc_cache);
  return s_adc_cache.temp;
}

int MHAL_ADC_CalibrateCurrent(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->adc == NULL ||
      motor_data.components.hal->adc->calibrate_offsets == NULL)
    return -1;

  motor_data.components.hal->adc->calibrate_offsets();
  return 0;
}
