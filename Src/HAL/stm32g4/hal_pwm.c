/**
 * @file hal_pwm.c
 * @brief PWM 硬件抽象层实现
 * @note 统一 HAL 架构：委托给 motor_data.components.hal->pwm
 *       motor_data 是静态初始化的，components.hal 在程序启动时即有效
 */

#include "hal_pwm.h"
#include "motor.h"
#include "config.h"

/**
 * @brief 注册 PWM 接口 (已废弃)
 * @note 统一 HAL 后不再需要注册，保留函数签名以兼容遗留代码
 */
int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface) {
  (void)interface; // 忽略参数，不再使用
  return 0;        // 总是返回成功
}

int MHAL_PWM_Init(void) {
  // motor_hal_g431.c 中的 PWM 接口没有 init 函数
  // 初始化由 MX_TIM1_Init() 在 main.c 中完成
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
  // 返回配置的 PWM 频率
  return PWM_FREQUENCY;
}

float MHAL_PWM_GetPeriod(void) {
  // 返回配置的 PWM 周期
  return CURRENT_MEASURE_PERIOD;
}
