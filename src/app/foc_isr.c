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
 * @file foc_isr.c
 * @brief Sampling, protection, control and observer scheduling at 20 kHz.
 */
#include "encoder_interface.h"
#include "adc_sample_guard.h"
#include "adc.h"
#include "application.h"
#include "board_configuration.h"
#include "encoder_failure_guard.h"
#include "main.h"
#include "motor_runtime.h"
#include "current_calibration.h"
#include "drive_state_machine.h"
#include "motor_sampling.h"
#include "safety_manager.h"
#include "telemetry_vofa.h"
#include "runtime_compensation.h"
#include "observer_adapter.h"
#include "pwm_interface.h"
#include "watchdog_supervisor.h"

static AdcSampleGuardState s_adc_sample_guard;
static EncoderFailureGuardState s_encoder_failure_guard;

static inline bool ISR_ADCSequenceComplete(ADC_HandleTypeDef *hadc) {
  return __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS) != 0u;
}

static inline bool ISR_ADCHasError(ADC_HandleTypeDef *hadc) {
  const uint32_t error_mask =
      HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_JQOVF | HAL_ADC_ERROR_INTERNAL;
  return (hadc->ErrorCode & error_mask) != 0u ||
         __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR) != 0u ||
         __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JQOVF) != 0u;
}

static inline bool ISR_ADCValidateFreshSample(ADC_HandleTypeDef *hadc) {
  AdcSampleRaw sample = {
      .ia = (uint16_t)HW_ADC_IA_HANDLE.Instance->HW_ADC_IA_JDR,
      .ib = (uint16_t)HW_ADC_IB_HANDLE.Instance->HW_ADC_IB_JDR,
      .ic = (uint16_t)HW_ADC_IC_HANDLE.Instance->HW_ADC_IC_JDR,
      .vbus = (uint16_t)HW_ADC_VBUS_HANDLE.Instance->HW_ADC_VBUS_JDR,
  };
  return AdcSampleGuard_Check(&s_adc_sample_guard, &sample,
                              ISR_ADCSequenceComplete(hadc),
                              ISR_ADCHasError(hadc)) == ADC_SAMPLE_GUARD_OK;
}
static inline void ISR_UpdateSensors(MOTOR_DATA *motor) {
  MotorSensorData sensor_data;
  ADC_ReadSensors(&sensor_data);
  motor->algo_input.Ia = sensor_data.i_a;
  motor->algo_input.Ib = sensor_data.i_b;
  motor->algo_input.Ic = sensor_data.i_c;
  motor->algo_input.Vbus = sensor_data.v_bus;
  motor->feedback.temperature = sensor_data.temp;
}
static inline bool ISR_UpdateEncoder(MOTOR_DATA *motor) {
  EncoderData enc_data;
  if (MHAL_Encoder_Update() != 0) {
    if (EncoderFailureGuard_Record(&s_encoder_failure_guard, false)) {
      MHAL_PWM_Disable();
      Safety_TriggerFault(FAULT_ENCODER_LOSS, motor, &g_ds402_state_machine);
    }
    return false;
  }
  (void)EncoderFailureGuard_Record(&s_encoder_failure_guard, true);
  MHAL_Encoder_GetData(&enc_data);
  /* Mechanical feedback uses turn and turn/s; electrical angle uses rad.
   * MIT control converts mechanical feedback back to rad and rad/s. */
  motor->feedback.position = enc_data.angle_rad / M_2PI;  /* [turn] */
  motor->feedback.velocity = enc_data.velocity_rad / M_2PI; /* [turn/s] */
  motor->feedback.phase_angle = enc_data.elec_angle; /* [rad] */
  /* Open-loop: let control module accumulate theta_elec freely */
  if (motor->state.Control_Mode != CONTROL_MODE_OPEN) {
    motor->algo_input.theta_elec = enc_data.elec_angle;
  }
  return true;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != HW_ADC_CURRENT.Instance)
    return;
  /* Sampling starts before every consumer is initialized.  The HAL has
   * already acknowledged this interrupt, so returning keeps the bridge off. */
  if (!App_IsFocRuntimeReady()) return;
  static uint16_t state_machine_tick_count = 0;
  static uint16_t torque_compensation_tick_count = 0;
#if SCOPE_LOG_ENABLED
  static uint16_t scope_log_tick_count = 0;
#endif
  if (!ISR_ADCValidateFreshSample(hadc)) {
    MHAL_PWM_Disable();
    if (AdcSampleGuard_ShouldFault(&s_adc_sample_guard))
      Safety_TriggerFault(FAULT_ADC_STALE, &motor_data,
                          &g_ds402_state_machine);
    return;
  }

  // Passive startup offsets are collected while all phase outputs stay off.
  CurrentCalib_UpdateStartup(&motor_data);
  // Fresh current, voltage, temperature and encoder feedback.
  ISR_UpdateSensors(&motor_data);
  if (!ISR_UpdateEncoder(&motor_data)) return;
  // Fast protection (20 kHz) precedes FSM and control.
  Safety_Update_Fast(&motor_data, &g_ds402_state_machine);
  // State transitions (1 kHz).
  if (++state_machine_tick_count >= STATE_MACHINE_DECIMATION) {
    state_machine_tick_count = 0;
    StateMachine_Update(&g_ds402_state_machine);
  }
  // Feedforward and cogging compensation (5 kHz).
  if (++torque_compensation_tick_count >= TORQUE_COMPENSATION_UPDATE_DECIMATION) {
    torque_compensation_tick_count = 0;
    Feedforward_Update(&motor_data);
    Motor_API_Cogging_Update(&motor_data);
  }
  // State-gated control produces this cycle's Valpha/Vbeta.
  Motor_RunControlCycle(&motor_data);
  // SMO must follow FOC to use this cycle's voltage output (20 kHz).
  static SMO_Observer_t smo_state;
  static bool init = false;
  if (!init) {
    SMO_Observer_Init(&smo_state);
    init = true;
  }
  smo_state.alpha = motor_data.advanced.smo_alpha;
  smo_state.beta = motor_data.advanced.smo_beta;
  SMO_Observer_Update(&smo_state, &motor_data);
  // Optional scope sampling at SCOPE_LOG_FREQUENCY_HZ.
#if SCOPE_LOG_ENABLED
  if (++scope_log_tick_count >= SCOPE_LOG_DECIMATION) {
    scope_log_tick_count = 0;
    Scope_Update();
  }
#endif
  WatchdogSupervisor_MarkFOC();
}
