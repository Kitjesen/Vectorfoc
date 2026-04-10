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

#include "motor_api.h"
#include "current_calib.h"
#include "flux_calib.h"
#include "control/control.h" // For CurrentLoop_UpdateGain, Control_Init
#include "control/cogging.h"
#include "control/feedforward.h"
#include "control/field_weakening.h"
#include "control/ladrc.h"
#include "hal_pwm.h" // For MHAL_PWM_Brake ( HAL)
#include "observer/smo_observer.h"
#include "param_access.h"
#include "param_table.h"
#include "rsls_calib.h"
#include "safety_control.h"
// External reference to the main motor data structure
extern MOTOR_DATA motor_data;
// =============================================================================
// Lifecycle / Management API
// =============================================================================
// Motor_API_Init removed - SMO is lazy-initialized in
// Motor_API_Observer_Update()
void Init_Motor_No_Calib(MOTOR_DATA *motor) {
  // 1. initparam ()
  Param_SystemInitOnce();
  // 2. init (, FOCstate)
  Control_Init(motor);
  // : PWM  HAL (motor_data.components.hal->pwm) ，
  //        Motor_PWM_Driver_Init()
  // 3. setstaterunningmode
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.Cs_State = CS_STATE_IDLE;
  // motor->state.State_Mode = STATE_MODE_RUNNING; // Legacy
  // Stay in SWITCH_ON_DISABLED at startup; demo task requests OPERATION_ENABLED
  // This prevents the velocity current loop from running with uncalibrated offsets
  // 5. paramupdatecurrentgain
  CurrentLoop_UpdateGain(motor);
  // 6. init LADRC speed/velocity
  LADRC_Init(&motor->ladrc_state, &motor->ladrc_config);
  // 7. paramupdate (inner loop)
  motor->params_updated = true;
}
void Init_Motor_Calib(MOTOR_DATA *motor) {
  // motor->components.encoder->calib_valid = false;
  motor->state.Sub_State =
      RSLS_CALIBRATING; //  Rs/Ls//pole pairs/encodercalibration
  motor->state.Cs_State = CS_MOTOR_R_START; //
}
void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t calibration_type) {
  // 1. PWMoutput (safety)
  MHAL_PWM_Brake();
  // 2. resetPIDintegral (start)
  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);
  // 2b. reset LADRC observerstate
  LADRC_Reset(&motor->ladrc_state);
  // 3. FOCstate
  FOC_Algorithm_ResetState(&motor->algo_state);
  CalibContext_Reset(&motor->calib_ctx);
  // 4. setcalibrationstate
  // 4. Select calibration mode
  motor->calib_type_requested = calibration_type;
  motor->last_calib_result = CALIB_IN_PROGRESS;
  switch (calibration_type) {
  case 1:
  case 2:
    motor->state.Sub_State = RSLS_CALIBRATING;
    RSLSCalib_Start(motor, &motor->calib_ctx);
    break;
  case 3: // Current offset calibration only
    motor->state.Sub_State = CURRENT_CALIBRATING;
    CurrentCalib_Start(motor, &motor->calib_ctx);
    break;
  case 4: // Flux calibration only (requires encoder already calibrated)
    motor->state.Sub_State = FLUX_CALIBRATING;
    FluxCalib_Start(motor, &motor->calib_ctx);
    break;
  case 5: // Anti-cogging calibration
    Motor_API_StartCoggingCalib(motor);
    return; // Cogging calib runs independently, skip DS402 state change
  default:
    motor->state.Sub_State = CURRENT_CALIBRATING;
    CurrentCalib_Start(motor, &motor->calib_ctx);
    break;
  }
  // 5. statemode ( FSM )
  // motor->state.State_Mode = STATE_MODE_DETECTING; // Legacy
  StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
}

void Motor_AbortCalibration(MOTOR_DATA *motor) {
  if (motor->state.Sub_State == SUB_STATE_IDLE)
    return;
  MHAL_PWM_Brake();
  CalibContext_Reset(&motor->calib_ctx);
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.Cs_State = CS_STATE_IDLE;
  motor->last_calib_result = CALIB_ABORTED;
  StateMachine_RequestState(&g_ds402_state_machine, STATE_SWITCH_ON_DISABLED);
}

uint8_t Motor_PreCalibCheck(MOTOR_DATA *motor, uint8_t *fail_mask) {
  uint8_t pass = 0;
  uint8_t fail = 0;

  // Bit 0: Bus voltage >= 18V
  if (motor->algo_input.Vbus >= 18.0f) {
    pass |= (1u << 0);
  } else {
    fail |= (1u << 0);
  }

  // Bit 1: Temperature < 80°C
  if (motor->feedback.temperature < 80.0f) {
    pass |= (1u << 1);
  } else {
    fail |= (1u << 1);
  }

  // Bit 2: Motor not in GUARD (fault) state
  if (motor->state.State_Mode != STATE_MODE_GUARD) {
    pass |= (1u << 2);
  } else {
    fail |= (1u << 2);
  }

  // Bit 3: Encoder not reporting hardware loss fault
  {
    uint32_t faults = Safety_GetActiveFaultBits();
    if (!(faults & FAULT_ENCODER_LOSS)) {
      pass |= (1u << 3);
    } else {
      fail |= (1u << 3);
    }
  }

  if (fail_mask != NULL)
    *fail_mask = fail;
  return pass;
}
void Motor_ClearFaults(MOTOR_DATA *motor) {
  if (motor->state.State_Mode == STATE_MODE_GUARD) {
    // 1. fault
    motor->state.Fault_State = FAULT_STATE_NORMAL;
    // 2. reset PID  LADRC ()
    PID_clear(&motor->IqPID);
    PID_clear(&motor->IdPID);
    PID_clear(&motor->VelPID);
    PID_clear(&motor->PosPID);
    LADRC_Reset(&motor->ladrc_state);
    // 3.  IDLE state
    motor->state.State_Mode = STATE_MODE_IDLE;
    // 4.  LED  (Assuming RGB_DisplayColorById is available via some
    // include, or need to verify) RGB_DisplayColorById(3); // ，
    // headers incomplete. Wait, led.h is in motor.c but not here. Let's include
    // it.
  }
}
// =============================================================================
// Tuning / Configuration Implementations
// =============================================================================
void Motor_API_ConfigSMO(float alpha, float beta) {
  motor_data.advanced.smo_alpha = alpha;
  motor_data.advanced.smo_beta = beta;
}
void Motor_API_ConfigFeedforward(float friction_coeff) {
  motor_data.advanced.ff_friction = friction_coeff;
}
void Motor_API_ConfigFieldWeakening(float max_current, float start_velocity) {
  motor_data.advanced.fw_max_current = max_current;
  motor_data.advanced.fw_start_velocity = start_velocity;
}
void Motor_API_ConfigCogging(bool enable) {
  motor_data.advanced.cogging_comp_enabled = enable ? 1.0f : 0.0f;
}
void Motor_API_StartCoggingCalib(MOTOR_DATA *motor) {
  CoggingComp_StartCalibration(motor);
}
void Motor_API_StopCoggingCalib(MOTOR_DATA *motor) {
  CoggingComp_StopCalibration(motor);
}
bool Motor_API_IsCoggingCalibrating(void) {
  return CoggingComp_IsCalibrating();
}
bool Motor_API_IsCoggingMapValid(void) { return CoggingComp_IsValid(); }
// =============================================================================
// Task Hook Implementations
// =============================================================================
void Motor_API_Observer_Update(MOTOR_DATA *motor) {
  static SMO_Observer_t smo_state;
  static bool init = false;
  if (!init) {
    SMO_Observer_Init(&smo_state);
    init = true;
  }
  smo_state.alpha = motor->advanced.smo_alpha;
  smo_state.beta = motor->advanced.smo_beta;
  SMO_Observer_Update(&smo_state, motor);
}
void Motor_API_Feedforward_Update(MOTOR_DATA *motor) {
  Feedforward_Params_t params;
  params.friction_coeff = motor->advanced.ff_friction;
  params.inertia = motor->Controller.inertia;
  Feedforward_Update(motor, &params);
}
void Motor_API_FieldWeakening_Update(MOTOR_DATA *motor) {
  FieldWeakening_Config_t cfg;
  cfg.max_weakening_current = motor->advanced.fw_max_current;
  cfg.start_velocity = motor->advanced.fw_start_velocity;
  FieldWeakening_Update(motor, &cfg);
}
void Motor_API_Cogging_Update(MOTOR_DATA *motor) {
  if (motor->advanced.cogging_calib_request > 0.5f &&
      !CoggingComp_IsCalibrating()) {
    CoggingComp_StartCalibration(motor);
    motor->advanced.cogging_calib_request = 0.0f;
  }
  CoggingComp_Update(motor);
}
