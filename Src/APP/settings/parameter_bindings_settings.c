// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "parameter_bindings_settings.h"

#include "config.h"
#include "fault_detection.h"
#include "motor.h"

static const ParamTargetBinding s_bindings[] = {
    {.index = PARAM_MOTOR_RS,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.parameters.Rs,
     .default_val = 0.5f},
    {.index = PARAM_MOTOR_LS,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.parameters.Ls,
     .default_val = 0.001f},
    {.index = PARAM_MOTOR_FLUX,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.parameters.flux,
     .default_val = 0.01f},
    {.index = PARAM_MOTOR_POLE_PAIRS,
     .type = PARAM_TYPE_UINT8,
     .target = &motor_data.parameters.pole_pairs,
     .default_val = (float)DEFAULT_POLE_PAIRS},
    {.index = PARAM_CUR_KP,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.current_ctrl_p_gain,
     .default_val = DEFAULT_CURRENT_P_GAIN},
    {.index = PARAM_CUR_KI,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.current_ctrl_i_gain,
     .default_val = DEFAULT_CURRENT_I_GAIN},
    {.index = PARAM_SPD_KP,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.VelPID.Kp,
     .default_val = DEFAULT_VEL_P_GAIN},
    {.index = PARAM_SPD_KI,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.VelPID.Ki,
     .default_val = DEFAULT_VEL_I_GAIN},
    {.index = PARAM_POS_KP,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.PosPID.Kp,
     .default_val = DEFAULT_POS_P_GAIN},
    {.index = PARAM_LIMIT_TORQUE,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.torque_limit,
     .default_val = DEFAULT_TORQUE_LIMIT},
    {.index = PARAM_LIMIT_CURRENT,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.current_limit,
     .default_val = DEFAULT_CURRENT_LIMIT},
    {.index = PARAM_LIMIT_SPEED,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.vel_limit,
     .default_val = DEFAULT_VEL_LIMIT},
    {.index = PARAM_VEL_MAX,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.traj_vel,
     .default_val = DEFAULT_TRAJ_VEL},
    {.index = PARAM_ACC_SET,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.traj_accel,
     .default_val = DEFAULT_TRAJ_ACCEL},
    {.index = PARAM_ACC_RAD,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.traj_decel,
     .default_val = DEFAULT_TRAJ_DECEL},
    {.index = PARAM_INERTIA,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.Controller.inertia,
     .default_val = DEFAULT_INERTIA},
    {.index = PARAM_CAN_ID,
     .type = PARAM_TYPE_UINT8,
     .target = &g_can_id,
     .default_val = (float)DEFAULT_CAN_ID},
    {.index = PARAM_CAN_BAUDRATE,
     .type = PARAM_TYPE_UINT8,
     .target = &g_can_baudrate,
     .default_val = (float)DEFAULT_CAN_BAUDRATE},
    {.index = PARAM_PROTOCOL_TYPE,
     .type = PARAM_TYPE_UINT8,
     .target = &g_protocol_type,
     .default_val = (float)DEFAULT_PROTOCOL_TYPE},
    {.index = PARAM_CAN_TIMEOUT,
     .type = PARAM_TYPE_UINT32,
     .target = &g_can_timeout_ms,
     .default_val = (float)DEFAULT_CAN_TIMEOUT_MS},
    {.index = PARAM_ZERO_STA,
     .type = PARAM_TYPE_UINT8,
     .target = &g_zero_sta,
     .default_val = (float)DEFAULT_ZERO_STA},
    {.index = PARAM_ADD_OFFSET,
     .type = PARAM_TYPE_FLOAT,
     .target = &g_add_offset,
     .default_val = DEFAULT_ADD_OFFSET},
    {.index = PARAM_DAMPER,
     .type = PARAM_TYPE_UINT8,
     .target = &g_damper_enable,
     .default_val = (float)DEFAULT_DAMPER_ENABLE},
    {.index = PARAM_RUN_MODE,
     .type = PARAM_TYPE_UINT8,
     .target = &g_run_mode,
     .default_val = (float)DEFAULT_RUN_MODE},
    {.index = PARAM_SMO_ALPHA,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.smo_alpha,
     .default_val = 0.1f},
    {.index = PARAM_SMO_BETA,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.smo_beta,
     .default_val = 0.1f},
    {.index = PARAM_FF_FRICTION,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.ff_friction,
     .default_val = 0.0f},
    {.index = PARAM_FW_MAX_CUR,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.fw_max_current,
     .default_val = 0.0f},
    {.index = PARAM_FW_START_VEL,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.fw_start_velocity,
     .default_val = 100.0f},
    {.index = PARAM_COGGING_EN,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.cogging_comp_enabled,
     .default_val = 0.0f},
    {.index = PARAM_COGGING_CALIB,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.advanced.cogging_calib_request,
     .default_val = 0.0f},
    {.index = PARAM_LADRC_ENABLE,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.ladrc_enable,
     .default_val = (float)DEFAULT_LADRC_ENABLE},
    {.index = PARAM_LADRC_OMEGA_O,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.ladrc_config.omega_o,
     .default_val = DEFAULT_LADRC_OMEGA_O},
    {.index = PARAM_LADRC_OMEGA_C,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.ladrc_config.omega_c,
     .default_val = DEFAULT_LADRC_OMEGA_C},
    {.index = PARAM_LADRC_B0,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.ladrc_config.b0,
     .default_val = DEFAULT_LADRC_B0},
    {.index = PARAM_LADRC_MAX_OUT,
     .type = PARAM_TYPE_FLOAT,
     .target = &motor_data.ladrc_config.max_output,
     .default_val = DEFAULT_LADRC_MAX_OUT},
};

static bool ParameterBindingsSettings_Resolve(void *context, uint16_t index,
                                              ParamType type,
                                              ParamTargetBinding *binding) {
  (void)context;
  if (binding == NULL) {
    return false;
  }

  switch (index) {
  case PARAM_OV_THRESHOLD: {
    DetectionConfig *detection_config = Detection_GetConfig();
    if (detection_config == NULL)
      return false;
    *binding = (ParamTargetBinding){
        .index = index,
        .type = type,
        .target = &detection_config->over_voltage_threshold,
        .default_val = FAULT_VBUS_OVERVOLT_V,
    };
    return true;
  }
  case PARAM_UV_THRESHOLD: {
    DetectionConfig *detection_config = Detection_GetConfig();
    if (detection_config == NULL)
      return false;
    *binding = (ParamTargetBinding){
        .index = index,
        .type = type,
        .target = &detection_config->under_voltage_threshold,
        .default_val = FAULT_VBUS_UNDERVOLT_V,
    };
    return true;
  }
  case PARAM_OC_THRESHOLD: {
    DetectionConfig *detection_config = Detection_GetConfig();
    if (detection_config == NULL)
      return false;
    *binding = (ParamTargetBinding){
        .index = index,
        .type = type,
        .target = &detection_config->over_current_threshold,
        .default_val = FAULT_OVER_CURRENT_A,
    };
    return true;
  }
  case PARAM_OT_THRESHOLD: {
    DetectionConfig *detection_config = Detection_GetConfig();
    if (detection_config == NULL)
      return false;
    *binding = (ParamTargetBinding){
        .index = index,
        .type = type,
        .target = &detection_config->over_temp_threshold,
        .default_val = FAULT_TEMP_ERROR_C,
    };
    return true;
  }
  default:
    break;
  }

  for (uint32_t i = 0U; i < sizeof(s_bindings) / sizeof(s_bindings[0]); ++i) {
    if (s_bindings[i].index == index) {
      *binding = s_bindings[i];
      return true;
    }
  }
  return false;
}

ParamResult ParameterBindingsSettings_Install(void) {
  return ParamTable_SetBindingAdapter(ParameterBindingsSettings_Resolve, NULL);
}
