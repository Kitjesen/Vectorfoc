/**
 * @file bsp_pwm_stm32g4.c
 * @brief STM32G4 PWM 驱动适配器
 * @note 实现 HAL_PWM 接口，适配 STM32G4 硬件
 */

#include "main.h"
#include "motor/hal/motor_pwm_driver.h"
#include "tim.h"

/* PWM 配置参数 */
#define PWM_TIMER htim1
#define PWM_ARR 2125        // ARR 值 (对应 20kHz @ 170MHz)
#define PWM_FREQUENCY 20000 // 20kHz

/* 内部函数声明 */
static void bsp_pwm_init(void);
static void bsp_pwm_set_duty(float Ta, float Tb, float Tc);
static void bsp_pwm_enable(void);
static void bsp_pwm_disable(void);
static void bsp_pwm_brake(void);
static uint32_t bsp_pwm_get_frequency(void);
static float bsp_pwm_get_period(void);

/* PWM 接口实现 */
static const HAL_PWM_Interface_t g_pwm_interface = {
    .init = bsp_pwm_init,
    .set_duty = bsp_pwm_set_duty,
    .enable = bsp_pwm_enable,
    .disable = bsp_pwm_disable,
    .brake = bsp_pwm_brake,
    .get_frequency = bsp_pwm_get_frequency,
    .get_period = bsp_pwm_get_period,
};

/**
 * @brief 初始化 STM32G4 PWM
 */
static void bsp_pwm_init(void) {
  /* STM32 HAL 已在 MX_TIM1_Init() 中初始化 */

  /* 启动 PWM 输出 */
  HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_3);

  /* 启动互补 PWM 输出 */
  HAL_TIMEx_PWMN_Start(&PWM_TIMER, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&PWM_TIMER, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&PWM_TIMER, TIM_CHANNEL_3);

  /* 初始占空比为 0 */
  bsp_pwm_set_duty(0.0f, 0.0f, 0.0f);

  /* Deadtime Sanity Check */
  if (MCPWM_DEADTIME_CLOCKS < 5 || MCPWM_DEADTIME_CLOCKS > 200) {
    // Error: Deadtime out of safe range (approx 30ns - 1.2us @ 168MHz)
    #include "error_config.h"
    #ifdef USE_ERROR_MANAGER
      #include "error_manager.h"
      #include "error_types.h"
      ERROR_REPORT(ERROR_HW_PWM_DEADTIME, "PWM deadtime out of range");
    #endif
    Error_Handler();
  }
}

/**
 * @brief 设置三相占空比
 */
static void bsp_pwm_set_duty(float Ta, float Tb, float Tc) {
  /* 限制占空比范围 [0, 1] */
  if (Ta < 0.0f)
    Ta = 0.0f;
  if (Ta > 1.0f)
    Ta = 1.0f;
  if (Tb < 0.0f)
    Tb = 0.0f;
  if (Tb > 1.0f)
    Tb = 1.0f;
  if (Tc < 0.0f)
    Tc = 0.0f;
  if (Tc > 1.0f)
    Tc = 1.0f;

  /* 转换为比较值 */
  uint16_t ccr_a = (uint16_t)(Ta * PWM_ARR);
  uint16_t ccr_b = (uint16_t)(Tb * PWM_ARR);
  uint16_t ccr_c = (uint16_t)(Tc * PWM_ARR);

  /* 设置比较值 */
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, TIM_CHANNEL_1, ccr_a);
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(&PWM_TIMER, TIM_CHANNEL_3, ccr_c);
}

/**
 * @brief 启用 PWM 输出
 */
static void bsp_pwm_enable(void) {
  HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&PWM_TIMER, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&PWM_TIMER, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&PWM_TIMER, TIM_CHANNEL_3);
}

/**
 * @brief 禁用 PWM 输出
 */
static void bsp_pwm_disable(void) {
  HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Stop(&PWM_TIMER, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&PWM_TIMER, TIM_CHANNEL_3);
}

/**
 * @brief 设置下桥臂导通（刹车）
 */
static void bsp_pwm_brake(void) {
  /* 设置所有占空比为 0，下桥臂导通 */
  bsp_pwm_set_duty(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 获取 PWM 频率
 */
static uint32_t bsp_pwm_get_frequency(void) { return PWM_FREQUENCY; }

/**
 * @brief 获取 PWM 周期
 */
static float bsp_pwm_get_period(void) {
  return 1000000.0f / PWM_FREQUENCY; // 返回周期 [us]
}

/**
 * @brief 注册 STM32G4 PWM 接口
 *
 * 在系统初始化时调用
 */
void BSP_PWM_STM32G4_Register(void) { HAL_PWM_Register(&g_pwm_interface); }
