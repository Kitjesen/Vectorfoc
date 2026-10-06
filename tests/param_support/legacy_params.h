// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/* Frozen compatibility reference from source-before.zip (2026-09-28).
 * Keep the old hand-written field transfer independent of the new binding table.
 * This fixture must not be regenerated as part of a normal test/build. */
#ifndef LEGACY_PARAMS_H
#define LEGACY_PARAMS_H

static const ParamEntry legacy_schema[] = {
  {0x2000, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "motor_rs", &motor_data.parameters.Rs, 0.0f, 10.0f, 0.5f, true},
  {0x2001, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "motor_ls", &motor_data.parameters.Ls, 0.0f, 0.01f, 0.001f, true},
  {0x2002, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "motor_flux", &motor_data.parameters.flux, 0.0f, 0.1f, 0.01f, true},
  {0x2003, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "motor_pole_pairs", &motor_data.parameters.pole_pairs, 1, 50, (float)DEFAULT_POLE_PAIRS, true},
  {0x2010, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "cur_kp", &motor_data.Controller.current_ctrl_p_gain, 0.0f, 100.0f, DEFAULT_CURRENT_P_GAIN, true},
  {0x2011, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "cur_ki", &motor_data.Controller.current_ctrl_i_gain, 0.0f, 1000.0f, DEFAULT_CURRENT_I_GAIN, true},
  {0x2012, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "spd_kp", &motor_data.VelPID.Kp, 0.0f, 100.0f, DEFAULT_VEL_P_GAIN, true},
  {0x2013, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "spd_ki", &motor_data.VelPID.Ki, 0.0f, 100.0f, DEFAULT_VEL_I_GAIN, true},
  {0x2014, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "pos_kp", &motor_data.PosPID.Kp, 0.0f, 100.0f, DEFAULT_POS_P_GAIN, true},
  {0x2020, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "limit_torque", &motor_data.Controller.torque_limit, 0.0f, 50.0f, DEFAULT_TORQUE_LIMIT, true},
  {0x2021, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "limit_current", &motor_data.Controller.current_limit, 0.0f, 50.0f, DEFAULT_CURRENT_LIMIT, true},
  {0x2022, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "limit_speed", &motor_data.Controller.vel_limit, 0.0f, 1000.0f, DEFAULT_VEL_LIMIT, true},
  {0x2030, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "vel_max", &motor_data.Controller.traj_vel, 0.0f, 1000.0f, DEFAULT_TRAJ_VEL, true},
  {0x2031, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "acc_set", &motor_data.Controller.traj_accel, 0.0f, 10000.0f, DEFAULT_TRAJ_ACCEL, true},
  {0x2032, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "acc_rad", &motor_data.Controller.traj_decel, 0.0f, 10000.0f, DEFAULT_TRAJ_DECEL, true},
  {0x2033, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "inertia", &motor_data.Controller.inertia, 0.0f, 1.0f, DEFAULT_INERTIA, true},
  {0x3000, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "can_id", &g_can_id, 1, 127, (float)DEFAULT_CAN_ID, true},
  {0x3001, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "can_baudrate", &g_can_baudrate, 0, 2, (float)DEFAULT_CAN_BAUDRATE, true},
  {0x3002, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "protocol_type", &g_protocol_type, 0, 2, (float)DEFAULT_PROTOCOL_TYPE, true},
  {0x3003, PARAM_TYPE_UINT32, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "can_timeout", &g_can_timeout_ms, 0, 10000, (float)DEFAULT_CAN_TIMEOUT_MS, true},
  {0x3010, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "zero_sta", &g_zero_sta, 0, 1, (float)DEFAULT_ZERO_STA, true},
  {0x3011, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "add_offset", &g_add_offset, -6.28f, 6.28f, DEFAULT_ADD_OFFSET, true},
  {0x3012, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "damper", &g_damper_enable, 0, 1, (float)DEFAULT_DAMPER_ENABLE, true},
  {0x3030, PARAM_TYPE_UINT8, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "run_mode", &g_run_mode, 0, 10, (float)DEFAULT_RUN_MODE, true},
  {0x3020, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ov_threshold", &s_config.over_voltage_threshold, 0.0f, 100.0f, FAULT_VBUS_OVERVOLT_V, true},
  {0x3021, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "uv_threshold", &s_config.under_voltage_threshold, 0.0f, 100.0f, FAULT_VBUS_UNDERVOLT_V, true},
  {0x3022, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "oc_threshold", &s_config.over_current_threshold, 0.0f, 200.0f, FAULT_OVER_CURRENT_A, true},
  {0x3023, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ot_threshold", &s_config.over_temp_threshold, 0.0f, 200.0f, FAULT_TEMP_ERROR_C, true},
  {0x3040, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "smo_alpha", &motor_data.advanced.smo_alpha, 0.0f, 10.0f, 0.1f, true},
  {0x3041, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "smo_beta", &motor_data.advanced.smo_beta, 0.0f, 10.0f, 0.1f, true},
  {0x3042, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ff_friction", &motor_data.advanced.ff_friction, 0.0f, 10.0f, 0.0f, true},
  {0x3043, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "fw_max_cur", &motor_data.advanced.fw_max_current, 0.0f, 20.0f, 0.0f, true},
  {0x3044, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "fw_start_vel", &motor_data.advanced.fw_start_velocity, 0.0f, 1000.0f, 100.0f, true},
  {0x3045, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "cogging_en", &motor_data.advanced.cogging_comp_enabled, 0.0f, 1.0f, 0.0f, true},
  {0x3046, PARAM_TYPE_FLOAT, PARAM_ATTR_RUNTIME, PARAM_ACCESS_RW, "cogging_calib", &motor_data.advanced.cogging_calib_request, 0.0f, 1.0f, 0.0f, false},
  {0x3050, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ladrc_en", &motor_data.ladrc_enable, 0.0f, 1.0f, (float)DEFAULT_LADRC_ENABLE, true},
  {0x3051, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ladrc_wo", &motor_data.ladrc_config.omega_o, 10.0f, 5000.0f, DEFAULT_LADRC_OMEGA_O, true},
  {0x3052, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ladrc_wc", &motor_data.ladrc_config.omega_c, 5.0f, 2000.0f, DEFAULT_LADRC_OMEGA_C, true},
  {0x3053, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ladrc_b0", &motor_data.ladrc_config.b0, 0.1f, 10000.0f, DEFAULT_LADRC_B0, true},
  {0x3054, PARAM_TYPE_FLOAT, PARAM_ATTR_PERSISTENT, PARAM_ACCESS_RW, "ladrc_max", &motor_data.ladrc_config.max_output, 0.1f, 200.0f, DEFAULT_LADRC_MAX_OUT, true},
};

static int check_legacy_layout(void) {
  CHECK(offsetof(FlashParamData, magic) == 0);
  CHECK(offsetof(FlashParamData, version) == 4);
  CHECK(offsetof(FlashParamData, crc32) == 8);
  CHECK(offsetof(FlashParamData, reserved) == 12);
  CHECK(offsetof(FlashParamData, generation) == 16);
  CHECK(offsetof(FlashParamData, committed) == 20);
  CHECK(offsetof(FlashParamData, motor_rs) == 24);
  CHECK(offsetof(FlashParamData, motor_ls) == 28);
  CHECK(offsetof(FlashParamData, motor_flux) == 32);
  CHECK(offsetof(FlashParamData, motor_pole_pairs) == 36);
  CHECK(offsetof(FlashParamData, cur_kp) == 37);
  CHECK(offsetof(FlashParamData, cur_ki) == 41);
  CHECK(offsetof(FlashParamData, spd_kp) == 45);
  CHECK(offsetof(FlashParamData, spd_ki) == 49);
  CHECK(offsetof(FlashParamData, pos_kp) == 53);
  CHECK(offsetof(FlashParamData, cur_filt_gain) == 57);
  CHECK(offsetof(FlashParamData, spd_filt_gain) == 61);
  CHECK(offsetof(FlashParamData, limit_torque) == 65);
  CHECK(offsetof(FlashParamData, limit_current) == 69);
  CHECK(offsetof(FlashParamData, limit_speed) == 73);
  CHECK(offsetof(FlashParamData, vel_max) == 77);
  CHECK(offsetof(FlashParamData, acc_set) == 81);
  CHECK(offsetof(FlashParamData, acc_rad) == 85);
  CHECK(offsetof(FlashParamData, inertia) == 89);
  CHECK(offsetof(FlashParamData, can_id) == 93);
  CHECK(offsetof(FlashParamData, can_baudrate) == 94);
  CHECK(offsetof(FlashParamData, protocol_type) == 95);
  CHECK(offsetof(FlashParamData, zero_sta) == 96);
  CHECK(offsetof(FlashParamData, add_offset) == 97);
  CHECK(offsetof(FlashParamData, damper) == 101);
  CHECK(offsetof(FlashParamData, can_timeout) == 102);
  CHECK(offsetof(FlashParamData, run_mode) == 106);
  CHECK(offsetof(FlashParamData, over_voltage_threshold) == 107);
  CHECK(offsetof(FlashParamData, under_voltage_threshold) == 111);
  CHECK(offsetof(FlashParamData, over_current_threshold) == 115);
  CHECK(offsetof(FlashParamData, over_temp_threshold) == 119);
  CHECK(offsetof(FlashParamData, smo_alpha) == 123);
  CHECK(offsetof(FlashParamData, smo_beta) == 127);
  CHECK(offsetof(FlashParamData, ff_friction) == 131);
  CHECK(offsetof(FlashParamData, fw_max_current) == 135);
  CHECK(offsetof(FlashParamData, fw_start_velocity) == 139);
  CHECK(offsetof(FlashParamData, cogging_comp_enabled) == 143);
  CHECK(offsetof(FlashParamData, reserved_data) == 147);
  CHECK(sizeof(FlashParamData) == 1715);
  return 0;
}

static void Legacy_Collect(FlashParamData *flash_data) {
  memset(flash_data, 0, sizeof(FlashParamData));
  float tmp_float;
  uint8_t tmp_uint8;
  if (Param_ReadFloat(PARAM_MOTOR_RS, &tmp_float) == PARAM_OK)
    flash_data->motor_rs = tmp_float;
  if (Param_ReadFloat(PARAM_MOTOR_LS, &tmp_float) == PARAM_OK)
    flash_data->motor_ls = tmp_float;
  if (Param_ReadFloat(PARAM_MOTOR_FLUX, &tmp_float) == PARAM_OK)
    flash_data->motor_flux = tmp_float;
  if (Param_ReadUint8(PARAM_MOTOR_POLE_PAIRS, &tmp_uint8) == PARAM_OK)
    flash_data->motor_pole_pairs = tmp_uint8;
  if (Param_ReadFloat(PARAM_CUR_KP, &tmp_float) == PARAM_OK)
    flash_data->cur_kp = tmp_float;
  if (Param_ReadFloat(PARAM_CUR_KI, &tmp_float) == PARAM_OK)
    flash_data->cur_ki = tmp_float;
  if (Param_ReadFloat(PARAM_SPD_KP, &tmp_float) == PARAM_OK)
    flash_data->spd_kp = tmp_float;
  if (Param_ReadFloat(PARAM_SPD_KI, &tmp_float) == PARAM_OK)
    flash_data->spd_ki = tmp_float;
  if (Param_ReadFloat(PARAM_POS_KP, &tmp_float) == PARAM_OK)
    flash_data->pos_kp = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_TORQUE, &tmp_float) == PARAM_OK)
    flash_data->limit_torque = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_CURRENT, &tmp_float) == PARAM_OK)
    flash_data->limit_current = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_SPEED, &tmp_float) == PARAM_OK)
    flash_data->limit_speed = tmp_float;
  if (Param_ReadFloat(PARAM_VEL_MAX, &tmp_float) == PARAM_OK)
    flash_data->vel_max = tmp_float;
  if (Param_ReadFloat(PARAM_ACC_SET, &tmp_float) == PARAM_OK)
    flash_data->acc_set = tmp_float;
  if (Param_ReadFloat(PARAM_ACC_RAD, &tmp_float) == PARAM_OK)
    flash_data->acc_rad = tmp_float;
  if (Param_ReadFloat(PARAM_INERTIA, &tmp_float) == PARAM_OK)
    flash_data->inertia = tmp_float;
  if (Param_ReadUint8(PARAM_CAN_ID, &tmp_uint8) == PARAM_OK)
    flash_data->can_id = tmp_uint8;
  if (Param_ReadUint8(PARAM_CAN_BAUDRATE, &tmp_uint8) == PARAM_OK)
    flash_data->can_baudrate = tmp_uint8;
  if (Param_ReadUint8(PARAM_PROTOCOL_TYPE, &tmp_uint8) == PARAM_OK)
    flash_data->protocol_type = tmp_uint8;
  if (Param_ReadUint8(PARAM_ZERO_STA, &tmp_uint8) == PARAM_OK)
    flash_data->zero_sta = tmp_uint8;
  if (Param_ReadFloat(PARAM_ADD_OFFSET, &tmp_float) == PARAM_OK)
    flash_data->add_offset = tmp_float;
  if (Param_ReadUint8(PARAM_DAMPER, &tmp_uint8) == PARAM_OK)
    flash_data->damper = tmp_uint8;
  if (Param_ReadUint8(PARAM_RUN_MODE, &tmp_uint8) == PARAM_OK)
    flash_data->run_mode = tmp_uint8;
  const ParamEntry *entry = NULL;
  if (Param_GetInfo(PARAM_CAN_TIMEOUT, &entry) == PARAM_OK && entry != NULL) {
    if (entry->type == PARAM_TYPE_UINT32) {
      flash_data->can_timeout = *(uint32_t *)entry->ptr;
    }
  }
  if (Param_ReadFloat(PARAM_OV_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_voltage_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_UV_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->under_voltage_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_OC_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_current_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_OT_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_temp_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_SMO_ALPHA, &tmp_float) == PARAM_OK)
    flash_data->smo_alpha = tmp_float;
  if (Param_ReadFloat(PARAM_SMO_BETA, &tmp_float) == PARAM_OK)
    flash_data->smo_beta = tmp_float;
  if (Param_ReadFloat(PARAM_FF_FRICTION, &tmp_float) == PARAM_OK)
    flash_data->ff_friction = tmp_float;
  if (Param_ReadFloat(PARAM_FW_MAX_CUR, &tmp_float) == PARAM_OK)
    flash_data->fw_max_current = tmp_float;
  if (Param_ReadFloat(PARAM_FW_START_VEL, &tmp_float) == PARAM_OK)
    flash_data->fw_start_velocity = tmp_float;
  if (Param_ReadFloat(PARAM_COGGING_EN, &tmp_float) == PARAM_OK)
    flash_data->cogging_comp_enabled = tmp_float;
}

static void Legacy_Restore(const FlashParamData *flash_data) {
  Param_WriteFloat(PARAM_MOTOR_RS, flash_data->motor_rs);
  Param_WriteFloat(PARAM_MOTOR_LS, flash_data->motor_ls);
  Param_WriteFloat(PARAM_MOTOR_FLUX, flash_data->motor_flux);
  Param_WriteUint8(PARAM_MOTOR_POLE_PAIRS, flash_data->motor_pole_pairs);
  Param_WriteFloat(PARAM_CUR_KP, flash_data->cur_kp);
  Param_WriteFloat(PARAM_CUR_KI, flash_data->cur_ki);
  Param_WriteFloat(PARAM_SPD_KP, flash_data->spd_kp);
  Param_WriteFloat(PARAM_SPD_KI, flash_data->spd_ki);
  Param_WriteFloat(PARAM_POS_KP, flash_data->pos_kp);
  Param_WriteFloat(PARAM_LIMIT_TORQUE, flash_data->limit_torque);
  Param_WriteFloat(PARAM_LIMIT_CURRENT, flash_data->limit_current);
  Param_WriteFloat(PARAM_LIMIT_SPEED, flash_data->limit_speed);
  Param_WriteFloat(PARAM_VEL_MAX, flash_data->vel_max);
  Param_WriteFloat(PARAM_ACC_SET, flash_data->acc_set);
  Param_WriteFloat(PARAM_ACC_RAD, flash_data->acc_rad);
  Param_WriteFloat(PARAM_INERTIA, flash_data->inertia);
  Param_WriteUint8(PARAM_CAN_ID, flash_data->can_id);
  Param_WriteUint8(PARAM_CAN_BAUDRATE, flash_data->can_baudrate);
  Param_WriteUint8(PARAM_PROTOCOL_TYPE, flash_data->protocol_type);
  Param_WriteUint8(PARAM_ZERO_STA, flash_data->zero_sta);
  Param_WriteFloat(PARAM_ADD_OFFSET, flash_data->add_offset);
  Param_WriteUint8(PARAM_DAMPER, flash_data->damper);
  Param_WriteUint8(PARAM_RUN_MODE, flash_data->run_mode);
  const ParamEntry *entry = NULL;
  if (Param_GetInfo(PARAM_CAN_TIMEOUT, &entry) == PARAM_OK && entry != NULL) {
    if (entry->type == PARAM_TYPE_UINT32) {
      *(uint32_t *)entry->ptr = flash_data->can_timeout;
    }
  }
  Param_WriteFloat(PARAM_OV_THRESHOLD, flash_data->over_voltage_threshold);
  Param_WriteFloat(PARAM_UV_THRESHOLD, flash_data->under_voltage_threshold);
  Param_WriteFloat(PARAM_OC_THRESHOLD, flash_data->over_current_threshold);
  Param_WriteFloat(PARAM_OT_THRESHOLD, flash_data->over_temp_threshold);
  Param_WriteFloat(PARAM_SMO_ALPHA, flash_data->smo_alpha);
  Param_WriteFloat(PARAM_SMO_BETA, flash_data->smo_beta);
  Param_WriteFloat(PARAM_FF_FRICTION, flash_data->ff_friction);
  Param_WriteFloat(PARAM_FW_MAX_CUR, flash_data->fw_max_current);
  Param_WriteFloat(PARAM_FW_START_VEL, flash_data->fw_start_velocity);
  Param_WriteFloat(PARAM_COGGING_EN, flash_data->cogging_comp_enabled);
}

#endif /* LEGACY_PARAMS_H */
