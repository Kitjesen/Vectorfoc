#include "motor_pwm_driver.h"
#include "main.h"
#include "motor/config.h"


extern TIM_HandleTypeDef htim1;

static void Motor_PWM_Init_Impl(void) {
  // HAL_TIM_PWM_Start is handled in Enable in this project context usually,
  // but the HAL Init might do hardware init if needed.
  // For now, we assume MX_TIM1_Init called in main.
}

static void Motor_PWM_SetDuty_Impl(float Ta, float Tb, float Tc) {
  // Ta,Tb,Tc ∈ [0,1]，PWM_ARR 来自 motor/config.h；通道映射 CH3=A, CH2=B, CH1=C
  // We replicate that mapping here.

  uint16_t ccr_a = (uint16_t)(Ta * PWM_ARR);
  uint16_t ccr_b = (uint16_t)(Tb * PWM_ARR);
  uint16_t ccr_c = (uint16_t)(Tc * PWM_ARR);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_c);
}

static void Motor_PWM_Enable_Impl(void) {
  // From Foc_Pwm_Start
  // Pre-load 50% duty?
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_ARR >> 1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_ARR >> 1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_ARR >> 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,
                    TIM_CHANNEL_4); // Channel 4? Kept from original code.

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static void Motor_PWM_Disable_Impl(void) {
  // From Foc_Pwm_Stop
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void Motor_PWM_Brake_Impl(void) {
  // From Foc_Pwm_LowSides
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

  // Ensure they are started
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static uint32_t Motor_PWM_GetFrequency_Impl(void) { return PWM_FREQUENCY; }

static float Motor_PWM_GetPeriod_Impl(void) { return CURRENT_MEASURE_PERIOD; }

static const HAL_PWM_Interface_t motor_pwm_interface = {
    .init = Motor_PWM_Init_Impl,
    .set_duty = Motor_PWM_SetDuty_Impl,
    .enable = Motor_PWM_Enable_Impl,
    .disable = Motor_PWM_Disable_Impl,
    .brake = Motor_PWM_Brake_Impl,
    .get_frequency = Motor_PWM_GetFrequency_Impl,
    .get_period = Motor_PWM_GetPeriod_Impl,
};

void Motor_PWM_Driver_Init(void) { MHAL_PWM_Register(&motor_pwm_interface); }
