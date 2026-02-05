/**
 * @file hal_pwm.h
 * @brief PWM 硬件抽象层接口
 * @note 提供统一的 PWM 接口，屏蔽底层硬件差异
 */

#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PWM 接口结构体
 *
 * 底层驱动需要实现这些函数指针，并注册到 HAL 层
 */
typedef struct {
  /**
   * @brief 初始化 PWM
   */
  void (*init)(void);

  /**
   * @brief 设置三相占空比
   * @param Ta a 相占空比 [0.0~1.0]
   * @param Tb b 相占空比 [0.0~1.0]
   * @param Tc c 相占空比 [0.0~1.0]
   */
  void (*set_duty)(float Ta, float Tb, float Tc);

  /**
   * @brief 启用 PWM 输出
   */
  void (*enable)(void);

  /**
   * @brief 禁用 PWM 输出
   */
  void (*disable)(void);

  /**
   * @brief 设置下桥臂导通（刹车）
   */
  void (*brake)(void);

  /**
   * @brief 获取 PWM 频率
   * @return PWM 频率 [Hz]
   */
  uint32_t (*get_frequency)(void);

  /**
   * @brief 获取 PWM 周期
   * @return PWM 周期 [us]
   */
  float (*get_period)(void);

} HAL_PWM_Interface_t;

/**
 * @brief 注册 PWM 接口
 *
 * 底层驱动在初始化时调用此函数，注册具体的实现
 *
 * @param interface PWM 接口实现
 * @return 0: 成功, -1: 失败
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
