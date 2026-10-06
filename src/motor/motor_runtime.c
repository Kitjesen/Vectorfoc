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

#include "motor_runtime.h"
#include "current_calibration.h"
#include "flux_calibration.h"
#include "status_led.h"
#include "motor_configuration.h"
#include "foc/control_dispatcher.h"
#include "pwm_interface.h"
#include "parameter_access.h"
#include "parameter_registry.h"
#include "algorithm/pid_controller.h"
#include "resistance_inductance_calibration.h"
#include "safety_manager.h"
#include "hardware_interface.h"
#include "foc/cogging_compensation.h"
#include "algorithm/ladrc_controller.h"
#include <stdlib.h>
/* DS402state */
extern StateMachine g_ds402_state_machine;
extern uint8_t g_can_id;
/**
 * @brief motor
 * @note config: 14 (14S Li-ion)
 *       - voltage: 14 × 3.7V = 51.8V
 *       - voltage: 14 × 4.2V = 58.8V
 *       - voltage: 14 × 3.0V = 42.0V
 */
extern MOTOR_DATA motor_data;
/**
 * @brief calibrationstate
 * : current -> / -> fluxcalibration
 */
static void MotorInitializeTask(MOTOR_DATA *motor) {
  CalibResult result;
  switch (motor->state.Sub_State) {
  case CURRENT_CALIBRATING:
    result = CurrentCalib_Update(motor, &motor->calib_ctx);
    if (result == CALIB_SUCCESS) {
      motor->last_calib_result = CALIB_SUCCESS;
      if (motor->calib_type_requested == 3) {
        // Current-only calibration: done, return to idle
        motor->state.Sub_State = SUB_STATE_IDLE;
        motor->state.State_Mode = STATE_MODE_RUNNING;
        Param_ScheduleSave();
      } else {
        Init_Motor_Calib(motor); //  RSLS calibration
      }
    } else if (result != CALIB_IN_PROGRESS) {
      motor->last_calib_result = result;
      motor->state.State_Mode = STATE_MODE_GUARD;
    }
    break;
  case RSLS_CALIBRATING:
    result = RSLSCalib_Update(motor, &motor->calib_ctx, CURRENT_SAMPLE_PERIOD_S);
    if (result == CALIB_SUCCESS) {
      motor->last_calib_result = CALIB_SUCCESS;
      motor->state.Sub_State = FLUX_CALIBRATING;
    } else if (result != CALIB_IN_PROGRESS) {
      motor->last_calib_result = result;
      motor->state.State_Mode = STATE_MODE_GUARD;
    }
    break;
  case FLUX_CALIBRATING:
    result = FluxCalib_Update(motor, &motor->calib_ctx);
    if (result == CALIB_SUCCESS) {
      motor->last_calib_result = CALIB_SUCCESS;
      motor->state.Sub_State = SUB_STATE_IDLE;
      motor->state.State_Mode = STATE_MODE_RUNNING;
      Param_ScheduleSave(); // calibrationdone，（ISRsafety）
    } else if (result != CALIB_IN_PROGRESS) {
      motor->last_calib_result = result;
      motor->state.State_Mode = STATE_MODE_GUARD; // calibration
    }
    break;
  default:
    break; // SUB_STATE_IDLE
  }
}
/**
 * @brief FOC state
 *  IDLE -> DETECTING -> RUNNING -> GUARD state
 */
void Motor_RunControlCycle(MOTOR_DATA *motor) {
  // 1.  FSM state
  MotorState fsm_state = StateMachine_GetState(&g_ds402_state_machine);
  // state, state
  static MotorState last_state = STATE_NOT_READY_TO_SWITCH_ON;
  // 2. state ()
  switch (fsm_state) {
  case STATE_OPERATION_ENABLED:
    motor->state.State_Mode = STATE_MODE_RUNNING;
    break;
  case STATE_CALIBRATING:
    motor->state.State_Mode = STATE_MODE_DETECTING;
    break;
  case STATE_FAULT:
  case STATE_FAULT_REACTION_ACTIVE:
    motor->state.State_Mode = STATE_MODE_GUARD;
    break;
  default:
    motor->state.State_Mode = STATE_MODE_IDLE;
    break;
  }
  // 3. state (Entry Action)
  if (fsm_state != last_state) {
    //  IDLE  GUARD state，
    if (motor->state.State_Mode == STATE_MODE_IDLE ||
        motor->state.State_Mode == STATE_MODE_GUARD) {
      PID_clear(&motor->IqPID);
      PID_clear(&motor->IdPID);
      PID_clear(&motor->VelPID);
      PID_clear(&motor->PosPID);
      FOC_Algorithm_ResetState(&motor->algo_state);
      /* Quick Stop keeps the active brake applied by the FSM entry action. */
      if (fsm_state != STATE_QUICK_STOP_ACTIVE) MHAL_PWM_Disable();
      if (!g_ds402_state_machine.auto_advance ||
          g_ds402_state_machine.target_state != STATE_OPERATION_ENABLED) {
        motor->advanced.cogging_calib_request = 0.0f;
        CoggingComp_StopCalibration(motor);
      }
    }
    last_state = fsm_state;
  }
  // 4.  (Do Action)
  switch (motor->state.State_Mode) {
  case STATE_MODE_RUNNING: // runningmode
    Control_RunCurrentCycle(motor);
    break;
  case STATE_MODE_DETECTING: // calibration/mode
    MotorInitializeTask(motor);
    if (motor->state.Sub_State == SUB_STATE_IDLE) {
      // Exit to Switch On Disabled
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
    // Check if calibration failed (Transitioned to GUARD by
    // MotorInitializeTask)
    else if (motor->state.State_Mode == STATE_MODE_GUARD) {
      // faultstate FSM
      StateMachine_EnterFault(&g_ds402_state_machine, FAULT_STALL_OVERLOAD);
    }
    break;
  case STATE_MODE_IDLE:
  case STATE_MODE_GUARD:
    //  ( Entry stop)
    break;
  default:
    break;
  }
}


/**
 * @brief safetyprotection (200Hz)
 * ////，driver LED
 */
void MotorGuardTask(MOTOR_DATA *motor) {
  // 1. runningsafety (200Hz)
  Safety_Update_Slow(motor, &g_ds402_state_machine);
  // 2. get
  uint32_t fault_bits = Safety_GetActiveFaultBits();
  // 3. LED state
  if (motor->state.State_Mode == STATE_MODE_IDLE ||
      motor->state.State_Mode == STATE_MODE_DETECTING) {
    static uint32_t blink_cnt = 0;  /* 1 Hz blink: 100 × 5 ms = 500 ms half-period */
    if (++blink_cnt >= 100u) { blink_cnt = 0u; }
    RGB_DisplayColorById(blink_cnt < 50u ? 9u : 7u); /* 9=on, 7=off */
  } else if (Safety_HasActiveFault()) {
    // protectionstate
    RGB_DisplayColorById(0); //  faultprotectionstate
    // motorstate GUARD
    if (motor->state.State_Mode != STATE_MODE_GUARD) {
      motor->state.State_Mode = STATE_MODE_GUARD;
      // fault Fault_State（，）
      // @deprecated  Safety_GetActiveFaultBits() getfault
      if (fault_bits & FAULT_OVER_VOLTAGE)
        motor->state.Fault_State = FAULT_STATE_OVER_VOLTAGE;
      else if (fault_bits & FAULT_UNDER_VOLTAGE)
        motor->state.Fault_State = FAULT_STATE_UNDER_VOLTAGE;
      else if (fault_bits & FAULT_OVER_CURRENT)
        motor->state.Fault_State = FAULT_STATE_OVER_CURRENT;
      else if (fault_bits & FAULT_OVER_TEMP)
        motor->state.Fault_State = FAULT_STATE_OVER_TEMPERATURE;
      else if (fault_bits & FAULT_STALL_OVERLOAD)
        motor->state.Fault_State = FAULT_STATE_SPEEDING;
      else if (fault_bits & FAULT_ENCODER_LOSS)
        motor->state.Fault_State = FAULT_STATE_ENCODER_LOSS;
    }
  } else if (motor->state.State_Mode == STATE_MODE_RUNNING) {
    RGB_DisplayColorById(3); //  normalrunning
  } else {
    /* GUARD without active fault — fast blink (100ms period) to signal issue */
    static uint32_t guard_cnt = 0;
    if (++guard_cnt >= 20u) { guard_cnt = 0u; }
    RGB_DisplayColorById(guard_cnt < 10u ? 0u : 7u);
  }
}

void Init_Motor_No_Calib(MOTOR_DATA *motor) {
  Param_SystemInitOnce();
  Control_Initialize(motor);
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.Cs_State = CS_STATE_IDLE;
  /* No power-on request: startup offsets and an explicit command are required. */
  Control_UpdateCurrentGains(motor);
  LADRC_Init(&motor->ladrc_state, &motor->ladrc_config);
  motor->params_updated = true;
}
void Init_Motor_Calib(MOTOR_DATA *motor) {
  motor->state.Sub_State = RSLS_CALIBRATING;
  motor->state.Cs_State = CS_MOTOR_R_START;
}
void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t calibration_type) {
  if (!motor) return;
  uint32_t irq_state = HAL_EnterCritical();
  if (!Safety_CanEnable(motor)) {
    StateMachine_RequestState(&g_ds402_state_machine, STATE_SWITCH_ON_DISABLED);
    HAL_ExitCritical(irq_state);
    return;
  }
  if (calibration_type == 5) {
    Motor_API_StartCoggingCalib(motor);
    HAL_ExitCritical(irq_state);
    return;
  }
  MHAL_PWM_Disable();
  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);
  LADRC_Reset(&motor->ladrc_state);
  FOC_Algorithm_ResetState(&motor->algo_state);
  CalibContext_Reset(&motor->calib_ctx);
  motor->calib_type_requested = calibration_type;
  motor->last_calib_result = CALIB_IN_PROGRESS;
  motor->state.Cs_State = CS_STATE_IDLE;
  switch (calibration_type) {
  case 1:
  case 2:
    motor->state.Sub_State = RSLS_CALIBRATING;
    RSLSCalib_Start(motor, &motor->calib_ctx); /* software preparation only */
    break;
  case 4:
    motor->state.Sub_State = FLUX_CALIBRATING;
    /* FluxCalib_Update starts voltage injection after CALIBRATING is accepted. */
    break;
  default:
    motor->state.Sub_State = CURRENT_CALIBRATING;
    motor->calib_ctx.current.is_initialized = false;
    break;
  }
  if (!StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING)) {
    motor->state.Sub_State = SUB_STATE_IDLE;
    motor->last_calib_result = CALIB_ABORTED;
  }
  HAL_ExitCritical(irq_state);
}
void Motor_AbortCalibration(MOTOR_DATA *motor) {
  if (!motor) return;
  uint32_t irq_state = HAL_EnterCritical();
  MHAL_PWM_Disable();
  CoggingComp_StopCalibration(motor);
  motor->advanced.cogging_calib_request = 0.0f;
  CalibContext_Reset(&motor->calib_ctx);
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.Cs_State = CS_STATE_IDLE;
  motor->last_calib_result = CALIB_ABORTED;
  StateMachine_RequestState(&g_ds402_state_machine, STATE_SWITCH_ON_DISABLED);
  HAL_ExitCritical(irq_state);
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
  if (!motor || !Safety_ClearFaults(&g_ds402_state_machine)) return;
  motor->state.Fault_State = FAULT_STATE_NORMAL;
  motor->state.State_Mode = STATE_MODE_IDLE;
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.Cs_State = CS_STATE_IDLE;
  motor->advanced.cogging_calib_request = 0.0f;
  CoggingComp_StopCalibration(motor);
  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);
  LADRC_Reset(&motor->ladrc_state);
}

void Motor_API_StartCoggingCalib(MOTOR_DATA *motor) {
  if (!motor) return;
  uint32_t irq_state = HAL_EnterCritical();
  motor->advanced.cogging_calib_request = 0.0f;
  if (StateMachine_RequestState(&g_ds402_state_machine, STATE_OPERATION_ENABLED))
    motor->advanced.cogging_calib_request = 1.0f;
  HAL_ExitCritical(irq_state);
}

/* Cogging updates retain state and safety checks before calibration. */
void Motor_API_Cogging_Update(MOTOR_DATA *motor) {
  MotorState state = StateMachine_GetState(&g_ds402_state_machine);
  if (state == STATE_FAULT || state == STATE_FAULT_REACTION_ACTIVE ||
      !CurrentCalib_IsReady() || Safety_HasActiveFault()) {
    motor->advanced.cogging_calib_request = 0.0f;
    CoggingComp_StopCalibration(motor);
    return;
  }
  if (motor->advanced.cogging_calib_request > 0.5f &&
      state == STATE_OPERATION_ENABLED && motor->state.State_Mode == STATE_MODE_RUNNING &&
      !CoggingComp_IsCalibrating()) {
    motor->advanced.cogging_calib_request = 0.0f;
    if (Safety_CanEnable(motor)) CoggingComp_StartCalibration(motor);
  }
  CoggingComp_Update(motor);
}
