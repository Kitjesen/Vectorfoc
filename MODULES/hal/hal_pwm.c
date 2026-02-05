/**
 * @file hal_pwm.c
 * @brief PWM 硬件抽象层实现
 */

#include "hal_pwm.h"
#include <stddef.h>

/* 全局 PWM 接口指针 */
static const HAL_PWM_Interface_t *g_pwm_interface = NULL;

/**
 * @brief 注册 PWM 接口
 */
int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface) {
  if (interface == NULL)
    return -1;

  g_pwm_interface = interface;
  return 0;
}

int MHAL_PWM_Init(void) {
  if (g_pwm_interface == NULL || g_pwm_interface->init == NULL)
    return -1;

  g_pwm_interface->init();
  return 0;
}

int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc) {
  if (g_pwm_interface == NULL || g_pwm_interface->set_duty == NULL)
    return -1;

  g_pwm_interface->set_duty(Ta, Tb, Tc);
  return 0;
}

int MHAL_PWM_Enable(void) {
  if (g_pwm_interface == NULL || g_pwm_interface->enable == NULL)
    return -1;

  g_pwm_interface->enable();
  return 0;
}

int MHAL_PWM_Disable(void) {
  if (g_pwm_interface == NULL || g_pwm_interface->disable == NULL)
    return -1;

  g_pwm_interface->disable();
  return 0;
}

int MHAL_PWM_Brake(void) {
  if (g_pwm_interface == NULL || g_pwm_interface->brake == NULL)
    return -1;

  g_pwm_interface->brake();
  return 0;
}

uint32_t MHAL_PWM_GetFrequency(void) {
  if (g_pwm_interface == NULL || g_pwm_interface->get_frequency == NULL)
    return 0;

  return g_pwm_interface->get_frequency();
}

float MHAL_PWM_GetPeriod(void) {
  if (g_pwm_interface == NULL || g_pwm_interface->get_period == NULL)
    return 0.0f;

  return g_pwm_interface->get_period();
}
