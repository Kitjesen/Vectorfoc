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

/**
 * @file hal_pwm.c
 * @brief PWM HAL wrapper
 *
 * 层次设计：HAL 层不依赖 ALGO 层 motor_data，
 * 由 app_init.c 通过 MHAL_PWM_SetHandle() 注入 Motor_HAL_Handle_t 指针。
 */
#include "hal_pwm.h"
#include "motor_hal_api.h"
#include "config.h"

static const Motor_HAL_Handle_t *s_hal = NULL;

void MHAL_PWM_SetHandle(const void *hal_handle)
{
    s_hal = (const Motor_HAL_Handle_t *)hal_handle;
}

int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface)
{
    (void)interface;
    return 0;
}

int MHAL_PWM_Init(void) { return 0; }

int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc)
{
    if (s_hal == NULL || s_hal->pwm == NULL || s_hal->pwm->set_duty == NULL)
        return -1;
    s_hal->pwm->set_duty(Ta, Tb, Tc);
    return 0;
}

int MHAL_PWM_Enable(void)
{
    if (s_hal == NULL || s_hal->pwm == NULL || s_hal->pwm->enable == NULL)
        return -1;
    s_hal->pwm->enable();
    return 0;
}

int MHAL_PWM_Disable(void)
{
    if (s_hal == NULL || s_hal->pwm == NULL || s_hal->pwm->disable == NULL)
        return -1;
    s_hal->pwm->disable();
    return 0;
}

int MHAL_PWM_Brake(void)
{
    if (s_hal == NULL || s_hal->pwm == NULL || s_hal->pwm->brake == NULL)
        return -1;
    s_hal->pwm->brake();
    return 0;
}

uint32_t MHAL_PWM_GetFrequency(void) { return PWM_FREQUENCY; }
float MHAL_PWM_GetPeriod(void) { return CURRENT_MEASURE_PERIOD; }
