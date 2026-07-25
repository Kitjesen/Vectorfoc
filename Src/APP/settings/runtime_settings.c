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

#include "runtime_settings.h"

#include "control/control.h"
#include "control/ladrc.h"
#include "fault_detection.h"
#include "hal_encoder.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "pid.h"

static void RuntimeSettings_ApplyLadrcConfig(void) {
  PID_clear(&motor_data.VelPID);
  LADRC_Init(&motor_data.ladrc_state, &motor_data.ladrc_config);
}

int RuntimeSettings_ApplyEncoderOffset(void) {
  return MHAL_Encoder_SetOffset(g_add_offset);
}

static void RuntimeSettings_ApplyRunMode(void) {
  CONTROL_MODE control_mode;
  if (Motor_RunModeToControlMode(g_run_mode, &control_mode)) {
    motor_data.state.Control_Mode = control_mode;
  }
}

static void RuntimeSettings_ApplyProtocol(void) {
  if (g_protocol_type <= (uint8_t)PROTOCOL_MIT) {
    Protocol_SetType((ProtocolType)g_protocol_type);
  }
}

static void RuntimeSettings_ApplyAll(void) {
  CurrentLoop_ApplyConfiguredGains(&motor_data);
  RuntimeSettings_ApplyLadrcConfig();
  (void)RuntimeSettings_ApplyEncoderOffset();
  Detection_SetCANTimeout(g_can_timeout_ms);
  RuntimeSettings_ApplyRunMode();
  RuntimeSettings_ApplyProtocol();
}

static void RuntimeSettings_ApplyOne(uint16_t index) {
  switch (index) {
  case PARAM_MOTOR_RS:
  case PARAM_MOTOR_LS:
  case PARAM_MOTOR_FLUX:
  case PARAM_MOTOR_POLE_PAIRS:
    motor_data.params_updated = true;
    break;
  case PARAM_CUR_KP:
  case PARAM_CUR_KI:
  case PARAM_LIMIT_CURRENT:
  case PARAM_LIMIT_SPEED:
    CurrentLoop_ApplyConfiguredGains(&motor_data);
    break;
  case PARAM_SPD_KP:
  case PARAM_SPD_KI:
    PID_clear(&motor_data.VelPID);
    break;
  case PARAM_POS_KP:
    PID_clear(&motor_data.PosPID);
    break;
  case PARAM_RUN_MODE:
    RuntimeSettings_ApplyRunMode();
    break;
  case PARAM_PROTOCOL_TYPE:
    RuntimeSettings_ApplyProtocol();
    break;
  case PARAM_ADD_OFFSET:
    (void)RuntimeSettings_ApplyEncoderOffset();
    break;
  case PARAM_CAN_TIMEOUT:
    Detection_SetCANTimeout(g_can_timeout_ms);
    break;
  case PARAM_LADRC_ENABLE:
  case PARAM_LADRC_OMEGA_O:
  case PARAM_LADRC_OMEGA_C:
  case PARAM_LADRC_B0:
  case PARAM_LADRC_MAX_OUT:
    RuntimeSettings_ApplyLadrcConfig();
    break;
  default:
    break;
  }
}

static void RuntimeSettings_ApplyParameter(void *context, uint16_t index) {
  (void)context;
  if (index == PARAM_RUNTIME_APPLY_ALL) {
    RuntimeSettings_ApplyAll();
    return;
  }
  RuntimeSettings_ApplyOne(index);
}

void RuntimeSettings_InstallAdapter(void) {
  Param_SetRuntimeApplyCallback(RuntimeSettings_ApplyParameter, NULL);
}
