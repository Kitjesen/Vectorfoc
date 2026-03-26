/**
 * @file hal_pwm.c
 * @brief PWM
 * @note  HAL ： motor_data.components.hal->pwm
 *       motor_data init，components.hal start
 */
#include "hal_pwm.h"
#include "motor.h"
#include "config.h"
/**
 * @brief  PWM  ()
 * @note  HAL ，
 */
int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface) {
  (void)interface; // param，
  return 0;        //
}
int MHAL_PWM_Init(void) {
  // motor_hal_g431.c  PWM  init
  // init MX_TIM1_Init()  main.c done
  return 0;
}
int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->pwm == NULL ||
      motor_data.components.hal->pwm->set_duty == NULL)
    return -1;
  motor_data.components.hal->pwm->set_duty(Ta, Tb, Tc);
  return 0;
}
int MHAL_PWM_Enable(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->pwm == NULL ||
      motor_data.components.hal->pwm->enable == NULL)
    return -1;
  motor_data.components.hal->pwm->enable();
  return 0;
}
int MHAL_PWM_Disable(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->pwm == NULL ||
      motor_data.components.hal->pwm->disable == NULL)
    return -1;
  motor_data.components.hal->pwm->disable();
  return 0;
}
int MHAL_PWM_Brake(void) {
  if (motor_data.components.hal == NULL ||
      motor_data.components.hal->pwm == NULL ||
      motor_data.components.hal->pwm->brake == NULL)
    return -1;
  motor_data.components.hal->pwm->brake();
  return 0;
}
uint32_t MHAL_PWM_GetFrequency(void) {
  // config PWM frequency
  return PWM_FREQUENCY;
}
float MHAL_PWM_GetPeriod(void) {
  // config PWM period
  return CURRENT_MEASURE_PERIOD;
}
