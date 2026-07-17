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
 * @file isr_foc.c
 * @brief FOCinterrupt (20kHz)
 * @note  motor_task.c
 */
#include "isr_foc.h"
#include "adc_sample_guard.h"
#include "adc.h"
#include "encoder_failure_guard.h"
#include "board_config.h"
#include "main.h"
#include "motor.h"
#include "motor_api.h"
#include "fsm.h"
#include "hal_pwm.h"
#include "motor_adc.h"
#include "safety_control.h"
#include "vofa.h"
#include <stdbool.h>
#include "../../SAFE/watchdog_supervisor.h"
#define ADJUST_EN 0
#define FSM_UPDATE_DIV MOTOR_DECIM_DIV(CONTROL_FREQ_HZ, FSM_UPDATE_HZ)
#define ADV_CONTROL_DIV MOTOR_DECIM_DIV(CONTROL_FREQ_HZ, ADV_CONTROL_HZ)
#define HS_LOG_DIV MOTOR_DECIM_DIV(CONTROL_FREQ_HZ, HS_LOG_HZ)
static EncoderFailureGuardState s_encoder_failure_guard;
/* ==========  ========== */
static inline void ISR_UpdateSensors(MOTOR_DATA *motor) {
  Motor_HAL_SensorData_t sensor_data;
  motor->components.hal->adc->update(&sensor_data);
  motor->algo_input.Ia = sensor_data.i_a;
  motor->algo_input.Ib = sensor_data.i_b;
  motor->algo_input.Ic = sensor_data.i_c;
  motor->algo_input.Vbus = sensor_data.v_bus;
  motor->feedback.temperature = sensor_data.temp;
}
static inline bool ISR_UpdateEncoder(MOTOR_DATA *motor) {
  const Motor_HAL_EncoderInterface_t *encoder =
      motor->components.hal != NULL ? motor->components.hal->encoder : NULL;
  if (encoder == NULL || encoder->update == NULL ||
      encoder->get_data == NULL) {
    if (EncoderFailureGuard_Record(&s_encoder_failure_guard, false)) {
      MHAL_PWM_Disable();
      Safety_TriggerFault(FAULT_ENCODER_LOSS, motor, &g_ds402_state_machine);
    }
    return false;
  }

  Motor_HAL_EncoderData_t enc_data = {0};
  if (encoder->set_pole_pairs != NULL && motor->parameters.pole_pairs > 0 &&
      motor->parameters.pole_pairs <= UINT8_MAX) {
    encoder->set_pole_pairs((uint8_t)motor->parameters.pole_pairs);
  }
  if (!encoder->update()) {
    if (EncoderFailureGuard_Record(&s_encoder_failure_guard, false)) {
      MHAL_PWM_Disable();
      Safety_TriggerFault(FAULT_ENCODER_LOSS, motor, &g_ds402_state_machine);
    }
    return false;
  }
  (void)EncoderFailureGuard_Record(&s_encoder_failure_guard, true);

  encoder->get_data(&enc_data);
  /* Core position and velocity use turns/turns-per-second. position_rad is
   * deliberately multi-turn; angle_rad remains available to calibration
   * and diagnostics as a wrapped single-turn angle. */
  motor->feedback.position = enc_data.position_rad / M_2PI;
  motor->feedback.velocity = enc_data.velocity_rad / M_2PI;
  motor->feedback.phase_angle = enc_data.elec_angle;
  /* Open-loop: let control module accumulate theta_elec freely */
  if (motor->state.Control_Mode != CONTROL_MODE_OPEN) {
    motor->algo_input.theta_elec = enc_data.elec_angle;
  }
  return true;
}

static inline void ISR_RunAdvancedControl(MOTOR_DATA *motor) {
  Motor_API_Feedforward_Update(motor);
  Motor_API_Cogging_Update(motor);
}
/* ========== interrupt ========== */
static AdcSampleGuardState s_adc_sample_guard;

static inline bool ISR_ADCSequenceComplete(ADC_HandleTypeDef *hadc) {
  return __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS) != 0u;
}

static inline bool ISR_ADCHasError(ADC_HandleTypeDef *hadc) {
  const uint32_t adc_error_mask = HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_JQOVF |
                                  HAL_ADC_ERROR_INTERNAL;
  return (hadc->ErrorCode & adc_error_mask) != 0u ||
         __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR) != 0u;
}

static inline AdcSampleRaw ISR_ReadRawAdcSample(void) {
  AdcSampleRaw sample = {
      .ia = (uint16_t)HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR,
      .ib = (uint16_t)HW_ADC_IB_HANDLE.Instance->HW_ADC_IB_JDR,
      .ic = (uint16_t)HW_ADC_IC_HANDLE.Instance->HW_ADC_IC_JDR,
      .vbus = (uint16_t)HW_ADC_VBUS_HANDLE.Instance->HW_ADC_VBUS_JDR,
  };
  return sample;
}

static inline bool ISR_ADCValidateFreshSample(ADC_HandleTypeDef *hadc) {
  AdcSampleRaw sample = ISR_ReadRawAdcSample();
  AdcSampleGuardStatus status = AdcSampleGuard_Check(
      &s_adc_sample_guard, &sample, ISR_ADCSequenceComplete(hadc),
      ISR_ADCHasError(hadc));
  return status == ADC_SAMPLE_GUARD_OK;
}

static inline void ISR_HandleInvalidAdcSample(void) {
  MHAL_PWM_Brake();
  if (AdcSampleGuard_ShouldFault(&s_adc_sample_guard)) {
    Safety_TriggerFault(FAULT_ADC_STALE, &motor_data, &g_ds402_state_machine);
  }
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != HW_ADC_CURRENT.Instance)
    return;
#if ADJUST_EN
  ISR_UpdateSensors(&motor_data);
#else
  static uint16_t s_fsm_counter = 0;
  static uint16_t s_adv_counter = 0;
#if HS_LOG_ENABLE
  static uint16_t s_log_counter = 0;
#endif
  if (!ISR_ADCValidateFreshSample(hadc)) {
    ISR_HandleInvalidAdcSample();
    return;
  }

  // 1. acquisition
  ISR_UpdateSensors(&motor_data);
  // 2. updateencoder
  if (!ISR_UpdateEncoder(&motor_data)) {
    return;
  }
  // 3. safety (20kHz)
  Safety_Update_Fast(&motor_data, &g_ds402_state_machine);
  // 4. stateupdate
  if (++s_fsm_counter >= FSM_UPDATE_DIV) {
    s_fsm_counter = 0;
    StateMachine_Update(&g_ds402_state_machine);
  }
  // 5.
  if (++s_adv_counter >= ADV_CONTROL_DIV) {
    s_adv_counter = 0;
    ISR_RunAdvancedControl(&motor_data);
  }
  // 6. FOC (produces this cycle's Valpha/Vbeta)
  MotorStateTask(&motor_data);
  // 7. Observer update after FOC so it uses the current cycle's Valpha/Vbeta,
  //    eliminating the one-cycle delay that degraded SMO angle estimation at speed.
  Motor_API_Observer_Update(&motor_data);
  // 8.
#if HS_LOG_ENABLE
  if (++s_log_counter >= HS_LOG_DIV) {
    s_log_counter = 0;
    Scope_Update();
  }
#endif
#endif
  WatchdogSupervisor_MarkFOC();
}
