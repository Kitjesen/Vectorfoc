/**
 * @file hal_pwm.h
 * @brief PWM
 * @note  PWM ，
 */
#ifndef HAL_PWM_H
#define HAL_PWM_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief PWM
 *
 * driver， HAL
 */
typedef struct {
  /**
   * @brief init PWM
   */
  void (*init)(void);
  /**
   * @brief setphaseduty cycle
   * @param Ta a phaseduty cycle [0.0~1.0]
   * @param Tb b phaseduty cycle [0.0~1.0]
   * @param Tc c phaseduty cycle [0.0~1.0]
   */
  void (*set_duty)(float Ta, float Tb, float Tc);
  /**
   * @brief  PWM output
   */
  void (*enable)(void);
  /**
   * @brief disable PWM output
   */
  void (*disable)(void);
  /**
   * @brief set（）
   */
  void (*brake)(void);
  /**
   * @brief get PWM frequency
   * @return PWM frequency [Hz]
   */
  uint32_t (*get_frequency)(void);
  /**
   * @brief get PWM period
   * @return PWM period [us]
   */
  float (*get_period)(void);
} HAL_PWM_Interface_t;
/**
 * @brief  PWM
 *
 * driverinit，
 *
 * @param interface PWM
 * @return 0: , -1:
 */
int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface);
int MHAL_PWM_Init(void);
int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc);
int MHAL_PWM_Enable(void);
int MHAL_PWM_Disable(void);
int MHAL_PWM_Brake(void);
uint32_t MHAL_PWM_GetFrequency(void);
float MHAL_PWM_GetPeriod(void);
#ifdef __cplusplus
}
#endif
#endif /* HAL_PWM_H */
