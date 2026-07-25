// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "control/control.h"
#include "control/ladrc.h"
#include "fault_detection.h"
#include "hal_encoder.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "pid.h"
#include "settings/runtime_settings.h"
#include <stdio.h>

MOTOR_DATA motor_data;
uint8_t g_protocol_type = PROTOCOL_VECTOR;
uint32_t g_can_timeout_ms = 1000U;
float g_add_offset = 0.0f;
uint8_t g_run_mode = 0U;

static ParamRuntimeApplyCallback s_runtime_callback;
static void *s_runtime_context;
static int s_current_gain_apply_count;
static int s_velocity_pid_clear_count;
static int s_position_pid_clear_count;
static int s_ladrc_init_count;
static int s_encoder_offset_apply_count;
static int s_encoder_offset_result;
static float s_encoder_offset;
static int s_timeout_apply_count;
static uint32_t s_timeout;
static int s_protocol_apply_count;
static ProtocolType s_protocol;

void Param_SetRuntimeApplyCallback(ParamRuntimeApplyCallback callback,
                                   void *context) {
  s_runtime_callback = callback;
  s_runtime_context = context;
}

void CurrentLoop_ApplyConfiguredGains(MOTOR_DATA *motor) {
  if (motor == &motor_data) {
    s_current_gain_apply_count++;
  }
}

void PID_clear(PidTypeDef *pid) {
  if (pid == &motor_data.VelPID) {
    s_velocity_pid_clear_count++;
  } else if (pid == &motor_data.PosPID) {
    s_position_pid_clear_count++;
  }
}

void LADRC_Init(LADRC_State_t *state, const LADRC_Config_t *config) {
  if (state == &motor_data.ladrc_state && config == &motor_data.ladrc_config) {
    s_ladrc_init_count++;
  }
}

int MHAL_Encoder_SetOffset(float offset) {
  s_encoder_offset_apply_count++;
  s_encoder_offset = offset;
  return s_encoder_offset_result;
}

void Detection_SetCANTimeout(uint32_t timeout_ms) {
  s_timeout_apply_count++;
  s_timeout = timeout_ms;
}

void Protocol_SetType(ProtocolType protocol) {
  s_protocol_apply_count++;
  s_protocol = protocol;
}

static void ResetState(void) {
  motor_data = (MOTOR_DATA){0};
  g_protocol_type = PROTOCOL_VECTOR;
  g_can_timeout_ms = 1000U;
  g_add_offset = 0.0f;
  g_run_mode = 0U;
  s_runtime_callback = NULL;
  s_runtime_context = (void *)1;
  s_current_gain_apply_count = 0;
  s_velocity_pid_clear_count = 0;
  s_position_pid_clear_count = 0;
  s_ladrc_init_count = 0;
  s_encoder_offset_apply_count = 0;
  s_encoder_offset_result = 0;
  s_encoder_offset = 0.0f;
  s_timeout_apply_count = 0;
  s_timeout = 0U;
  s_protocol_apply_count = 0;
  s_protocol = PROTOCOL_VECTOR;
}

static int TestInstallAndSingleParameterMapping(void) {
  ResetState();
  RuntimeSettings_InstallAdapter();
  if (s_runtime_callback == NULL || s_runtime_context != NULL) {
    return 1;
  }

  s_runtime_callback(s_runtime_context, PARAM_MOTOR_RS);
  if (!motor_data.params_updated) {
    return 1;
  }

  s_runtime_callback(s_runtime_context, PARAM_CUR_KP);
  s_runtime_callback(s_runtime_context, PARAM_SPD_KP);
  s_runtime_callback(s_runtime_context, PARAM_POS_KP);
  s_runtime_callback(s_runtime_context, PARAM_LADRC_B0);
  if (s_current_gain_apply_count != 1 || s_velocity_pid_clear_count != 2 ||
      s_position_pid_clear_count != 1 || s_ladrc_init_count != 1) {
    return 1;
  }

  g_add_offset = 1.25f;
  g_can_timeout_ms = 321U;
  g_protocol_type = PROTOCOL_MIT;
  g_run_mode = 3U;
  s_runtime_callback(s_runtime_context, PARAM_ADD_OFFSET);
  s_runtime_callback(s_runtime_context, PARAM_CAN_TIMEOUT);
  s_runtime_callback(s_runtime_context, PARAM_PROTOCOL_TYPE);
  s_runtime_callback(s_runtime_context, PARAM_RUN_MODE);

  g_protocol_type = (uint8_t)(PROTOCOL_MIT + 1U);
  s_runtime_callback(s_runtime_context, PARAM_PROTOCOL_TYPE);

  return s_encoder_offset_apply_count != 1 || s_encoder_offset != 1.25f ||
         s_timeout_apply_count != 1 || s_timeout != 321U ||
         s_protocol_apply_count != 1 || s_protocol != PROTOCOL_MIT ||
         motor_data.state.Control_Mode != CONTROL_MODE_TORQUE;
}

static int TestBulkMappingAndEarlyEncoderOffset(void) {
  ResetState();
  RuntimeSettings_InstallAdapter();
  g_add_offset = -0.5f;
  g_can_timeout_ms = 777U;
  g_protocol_type = PROTOCOL_CANOPEN;
  g_run_mode = 1U;

  s_runtime_callback(s_runtime_context, PARAM_RUNTIME_APPLY_ALL);
  if (s_current_gain_apply_count != 1 || s_velocity_pid_clear_count != 1 ||
      s_ladrc_init_count != 1 || s_encoder_offset_apply_count != 1 ||
      s_encoder_offset != -0.5f || s_timeout_apply_count != 1 ||
      s_timeout != 777U || s_protocol_apply_count != 1 ||
      s_protocol != PROTOCOL_CANOPEN ||
      motor_data.state.Control_Mode != CONTROL_MODE_POSITION_RAMP) {
    return 1;
  }

  s_encoder_offset_result = -7;
  g_add_offset = 2.0f;
  if (RuntimeSettings_ApplyEncoderOffset() != -7 ||
      s_encoder_offset_apply_count != 2 || s_encoder_offset != 2.0f) {
    return 1;
  }
  return 0;
}

int main(void) {
  if (TestInstallAndSingleParameterMapping()) {
    printf("FAIL runtime settings single parameter mapping\n");
    return 1;
  }
  if (TestBulkMappingAndEarlyEncoderOffset()) {
    printf("FAIL runtime settings bulk mapping\n");
    return 1;
  }
  printf("Runtime settings adapter tests passed\n");
  return 0;
}
