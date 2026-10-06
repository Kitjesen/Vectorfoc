// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pwm_interface.h"
#include "tim.h"
#include "motor_runtime.h"
#include "current_calibration.h"
#include "safety_manager.h"
#include "board_configuration.h"

static const uint32_t phase_channels[] = {HW_PWM_CH_U, HW_PWM_CH_V, HW_PWM_CH_W};
static bool PowerStateAccepted(void) {
  MotorState state = StateMachine_GetState(&g_ds402_state_machine);
  return (state == STATE_OPERATION_ENABLED || state == STATE_CALIBRATING ||
          state == STATE_QUICK_STOP_ACTIVE) &&
         CurrentCalib_IsReady() && !Safety_HasActiveFault();
}
int MHAL_PWM_Disable(void) {
  int status = 0;
  for (unsigned i = 0; i < 3; ++i) {
    if (HAL_TIM_PWM_Stop(&HW_PWM_TIMER, phase_channels[i]) != HAL_OK) status = -1;
    if (HAL_TIMEx_PWMN_Stop(&HW_PWM_TIMER, phase_channels[i]) != HAL_OK) status = -1;
  }
  return status; /* CH4 remains active, retaining the ADC/safety timebase. */
}
int MHAL_PWM_StartSampling(void) {
  if (TIM_CHANNEL_STATE_GET(&HW_PWM_TIMER, HW_PWM_CH_TRIG) == HAL_TIM_CHANNEL_STATE_BUSY)
    return 0;
  if (MHAL_PWM_Disable() != 0) return -1;
  return HAL_TIM_PWM_Start(&HW_PWM_TIMER, HW_PWM_CH_TRIG) == HAL_OK ? 0 : -1;
}
int MHAL_PWM_SetDuty(float a, float b, float c) {
  uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&HW_PWM_TIMER);
  /* Physical U/V/W phase mapping is supplied by the selected board. */
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_U, (uint16_t)(a * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_V, (uint16_t)(b * arr));
  __HAL_TIM_SET_COMPARE(&HW_PWM_TIMER, HW_PWM_CH_W, (uint16_t)(c * arr));
  return 0;
}
int MHAL_PWM_Enable(void) {
  if (!PowerStateAccepted() ||
      TIM_CHANNEL_STATE_GET(&HW_PWM_TIMER, HW_PWM_CH_TRIG) != HAL_TIM_CHANNEL_STATE_BUSY)
    return -1;
  for (unsigned i = 0; i < 3; ++i) {
    uint32_t channel = phase_channels[i];
    if (TIM_CHANNEL_STATE_GET(&HW_PWM_TIMER, channel) != HAL_TIM_CHANNEL_STATE_BUSY &&
        HAL_TIM_PWM_Start(&HW_PWM_TIMER, channel) != HAL_OK) goto failed;
    if (TIM_CHANNEL_N_STATE_GET(&HW_PWM_TIMER, channel) != HAL_TIM_CHANNEL_STATE_BUSY &&
        HAL_TIMEx_PWMN_Start(&HW_PWM_TIMER, channel) != HAL_OK) goto failed;
  }
  return 0;
failed:
  MHAL_PWM_Disable();
  return -1;
}
int MHAL_PWM_Brake(void) {
  if (!PowerStateAccepted()) return -1;
  MHAL_PWM_SetDuty(0.0f, 0.0f, 0.0f);
  return MHAL_PWM_Enable();
}
