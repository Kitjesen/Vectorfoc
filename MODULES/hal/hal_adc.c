/**
 * @file hal_adc.c
 * @brief ADC 硬件抽象层实现
 */

#include "hal_adc.h"
#include <stddef.h>

/* 全局 ADC 接口指针 */
static const HAL_ADC_Interface_t *g_adc_interface = NULL;

/**
 * @brief 注册 ADC 接口
 */
int MHAL_ADC_Register(const HAL_ADC_Interface_t *interface) {
  if (interface == NULL)
    return -1;

  g_adc_interface = interface;
  return 0;
}

int MHAL_ADC_Init(void) {
  if (g_adc_interface == NULL || g_adc_interface->init == NULL)
    return -1;

  g_adc_interface->init();
  return 0;
}

int MHAL_ADC_Start(void) {
  if (g_adc_interface == NULL || g_adc_interface->start == NULL)
    return -1;

  g_adc_interface->start();
  return 0;
}

int MHAL_ADC_Stop(void) {
  if (g_adc_interface == NULL || g_adc_interface->stop == NULL)
    return -1;

  g_adc_interface->stop();
  return 0;
}

int MHAL_ADC_GetCurrent(float *Ia, float *Ib, float *Ic) {
  if (g_adc_interface == NULL || g_adc_interface->get_current == NULL)
    return -1;

  g_adc_interface->get_current(Ia, Ib, Ic);
  return 0;
}

float MHAL_ADC_GetVbus(void) {
  if (g_adc_interface == NULL || g_adc_interface->get_vbus == NULL)
    return 0.0f;

  return g_adc_interface->get_vbus();
}

float MHAL_ADC_GetTemperature(void) {
  if (g_adc_interface == NULL || g_adc_interface->get_temperature == NULL)
    return 0.0f;

  return g_adc_interface->get_temperature();
}

int MHAL_ADC_CalibrateCurrent(void) {
  if (g_adc_interface == NULL || g_adc_interface->calibrate_current == NULL)
    return -1;

  g_adc_interface->calibrate_current();
  return 0;
}
