#ifndef STARTUP_TEST_TIM_H
#define STARTUP_TEST_TIM_H
#ifdef STARTUP_TEST
#include "mock_hal_types.h"
#endif
#include "adc.h"
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 1u
#define TIM_CHANNEL_3 2u
#define TIM_CHANNEL_4 3u
#define HAL_TIM_CHANNEL_STATE_READY 0u
#define HAL_TIM_CHANNEL_STATE_BUSY 1u
#ifndef MOCK_COMMON_H
typedef struct {
  uint32_t arr, compare[4];
  unsigned main_state[4], complement_state[4];
} TIM_HandleTypeDef;
#endif
extern TIM_HandleTypeDef htim1;
#define TIM_CHANNEL_STATE_GET(timer, channel) ((timer)->main_state[channel])
#define TIM_CHANNEL_N_STATE_GET(timer, channel) ((timer)->complement_state[channel])
#define __HAL_TIM_GET_AUTORELOAD(timer) ((timer)->arr)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare[channel] = (value))
#ifndef HAL_TIM_PWM_Start
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
#endif
