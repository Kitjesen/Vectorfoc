// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

/**
 * @file hal_pwm.c
 * @brief Typed adapter from the legacy MHAL API to a replaceable PWM port.
 */
#include "hal_pwm.h"

#include "config.h"
#include <stddef.h>

static const Motor_HAL_PwmInterface_t *s_pwm;

int MHAL_PWM_Bind(const Motor_HAL_PwmInterface_t *interface) {
  s_pwm = interface;
  return interface != NULL ? 0 : -1;
}

int MHAL_PWM_Register(const HAL_PWM_Interface_t *interface) {
  (void)interface;
  return -1;
}

int MHAL_PWM_Init(void) { return s_pwm != NULL ? 0 : -1; }

int MHAL_PWM_StartSampling(void) {
  if (s_pwm == NULL || s_pwm->start_sampling == NULL) {
    return -1;
  }
  return s_pwm->start_sampling() ? 0 : -1;
}

int MHAL_PWM_SetDuty(float Ta, float Tb, float Tc) {
  if (s_pwm == NULL || s_pwm->set_duty == NULL) {
    return -1;
  }
  s_pwm->set_duty(Ta, Tb, Tc);
  return 0;
}

int MHAL_PWM_Enable(void) {
  if (s_pwm == NULL || s_pwm->enable == NULL) {
    return -1;
  }
  return s_pwm->enable() ? 0 : -1;
}

int MHAL_PWM_Disable(void) {
  if (s_pwm == NULL || s_pwm->disable == NULL) {
    return -1;
  }
  s_pwm->disable();
  return 0;
}

int MHAL_PWM_Brake(void) {
  if (s_pwm == NULL || s_pwm->brake == NULL) {
    return -1;
  }
  s_pwm->brake();
  return 0;
}

uint32_t MHAL_PWM_GetFrequency(void) { return PWM_FREQUENCY; }
float MHAL_PWM_GetPeriod(void) { return CURRENT_MEASURE_PERIOD; }
