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
 * @file param_table.c
 * @brief ㄥ?- ユ
 */
#include "param_table.h"
#include "fault_detection.h"
#include "motor.h"
#include "config.h"
/* 编码器驱动头文件已从此处移除：param_table.c 不需要直接访问编码器内部字段。
 * 编码器相关参数（pole_pairs 等）通过 motor_data.parameters 访问，
 * 已由 #include "motor.h" 提供完整定义。 */
#include <string.h>
/* ============================================================================
 * ㄥ?
 * ============================================================================
 */
static const ParamEntry s_param_table[] = {
    /* ===  === */
    {.index = PARAM_MOTOR_RS,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_rs",
     .ptr = &motor_data.parameters.Rs,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.5f,
     .need_save = true},
    {.index = PARAM_MOTOR_LS,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_ls",
     .ptr = &motor_data.parameters.Ls,
     .min = 0.0f,
     .max = 0.01f,
     .default_val = 0.001f,
     .need_save = true},
    {.index = PARAM_MOTOR_FLUX,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_flux",
     .ptr = &motor_data.parameters.flux,
     .min = 0.0f,
     .max = 0.1f,
     .default_val = 0.01f,
     .need_save = true},
    {.index = PARAM_MOTOR_POLE_PAIRS,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_pole_pairs",
     .ptr = &motor_data.parameters.pole_pairs,
     .min = 1,
     .max = 50,
     .default_val = (float)DEFAULT_POLE_PAIRS,
     .need_save = true},
    /* === PID === */
    {.index = PARAM_CUR_KP,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "cur_kp",
     .ptr = &motor_data.Controller.current_ctrl_p_gain,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = DEFAULT_CURRENT_P_GAIN,
     .need_save = true},
    {.index = PARAM_CUR_KI,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "cur_ki",
     .ptr = &motor_data.Controller.current_ctrl_i_gain,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = DEFAULT_CURRENT_I_GAIN,
     .need_save = true},
    {.index = PARAM_SPD_KP,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "spd_kp",
     .ptr = &motor_data.VelPID.Kp,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = DEFAULT_VEL_P_GAIN,
     .need_save = true},
    {.index = PARAM_SPD_KI,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "spd_ki",
     .ptr = &motor_data.VelPID.Ki,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = DEFAULT_VEL_I_GAIN,
     .need_save = true},
    {.index = PARAM_POS_KP, // Same as PARAM_LOC_KP
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "pos_kp",
     .ptr = &motor_data.PosPID.Kp,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = DEFAULT_POS_P_GAIN,
     .need_save = true},
    // Note: filter_alpha removed - PidTypeDef doesn't have this member
    // CUR_FILT_GAIN and SPD_FILT_GAIN parameters disabled
    /* ===  === */
    {.index = PARAM_LIMIT_TORQUE,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "limit_torque",
     .ptr = &motor_data.Controller.torque_limit,
     .min = 0.0f,
     .max = 50.0f,
     .default_val = DEFAULT_TORQUE_LIMIT,
     .need_save = true},
    {.index = PARAM_LIMIT_CURRENT, // Same as PARAM_LIMIT_CUR
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "limit_current",
     .ptr = &motor_data.Controller.current_limit,
     .min = 0.0f,
     .max = 50.0f,
     .default_val = DEFAULT_CURRENT_LIMIT,
     .need_save = true},
    {.index = PARAM_LIMIT_SPEED, // Same as PARAM_LIMIT_SPD
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "limit_speed",
     .ptr = &motor_data.Controller.vel_limit,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = DEFAULT_VEL_LIMIT,
     .need_save = true},
    /* === /″ === */
    {.index = PARAM_VEL_MAX,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "vel_max",
     .ptr = &motor_data.Controller.traj_vel,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = DEFAULT_TRAJ_VEL,
     .need_save = true},
    {.index = PARAM_ACC_SET,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "acc_set",
     .ptr = &motor_data.Controller.traj_accel,
     .min = 0.0f,
     .max = 10000.0f,
     .default_val = DEFAULT_TRAJ_ACCEL,
     .need_save = true},
    {.index = PARAM_ACC_RAD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "acc_rad",
     .ptr = &motor_data.Controller.traj_decel,
     .min = 0.0f,
     .max = 10000.0f,
     .default_val = DEFAULT_TRAJ_DECEL,
     .need_save = true},
    {.index = PARAM_INERTIA,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "inertia",
     .ptr = &motor_data.Controller.inertia,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = DEFAULT_INERTIA, // Default 0 (Disable FF)
     .need_save = true},
    /* === CAN === */
    {.index = PARAM_CAN_ID,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "can_id",
     .ptr = &g_can_id,
     .min = 1,
     .max = 127,
     .default_val = (float)DEFAULT_CAN_ID,
     .need_save = true},
    {.index = PARAM_CAN_BAUDRATE,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "can_baudrate",
     .ptr = &g_can_baudrate,
     .min = 0,
     .max = 2,
     .default_val = (float)DEFAULT_CAN_BAUDRATE, // 0=1M, 1=500K, 2=250K
     .need_save = true},
    {.index = PARAM_PROTOCOL_TYPE,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "protocol_type",
     .ptr = &g_protocol_type,
     .min = 0,
     .max = 2,
     .default_val = (float)DEFAULT_PROTOCOL_TYPE, // 0=, 1=CANopen, 2=MIT
     .need_save = true},
    {.index = PARAM_CAN_TIMEOUT,
     .type = PARAM_TYPE_UINT32,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "can_timeout",
     .ptr = &g_can_timeout_ms,
     .min = 0,
     .max = 10000,
     .default_val = (float)DEFAULT_CAN_TIMEOUT_MS, // 1000ms
     .need_save = true},
    /* ===  === */
    {.index = PARAM_ZERO_STA,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "zero_sta",
     .ptr = &g_zero_sta,
     .min = 0,
     .max = 1,
     .default_val = (float)DEFAULT_ZERO_STA, // 0: 0~2, 1: -~
     .need_save = true},
    {.index = PARAM_ADD_OFFSET,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "add_offset",
     .ptr = &g_add_offset,
     .min = -6.28f,
     .max = 6.28f,
     .default_val = DEFAULT_ADD_OFFSET,
     .need_save = true},
    {.index = PARAM_DAMPER,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "damper",
     .ptr = &g_damper_enable,
     .min = 0,
     .max = 1,
     .default_val = (float)DEFAULT_DAMPER_ENABLE,
     .need_save = true},
    {.index = PARAM_RUN_MODE,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "run_mode",
     .ptr = &g_run_mode,
     .min = 0,
     .max = 10,
     .default_val = (float)DEFAULT_RUN_MODE,
     .need_save = true},
    /* ===  === */
    {.index = PARAM_OV_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ov_threshold",
     .ptr = &s_config.over_voltage_threshold,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = FAULT_VBUS_OVERVOLT_V,
     .need_save = true},
    {.index = PARAM_UV_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "uv_threshold",
     .ptr = &s_config.under_voltage_threshold,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = FAULT_VBUS_UNDERVOLT_V,
     .need_save = true},
    {.index = PARAM_OC_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "oc_threshold",
     .ptr = &s_config.over_current_threshold,
     .min = 0.0f,
     .max = 200.0f,
     .default_val = FAULT_OVER_CURRENT_A,
     .need_save = true},
    {.index = PARAM_OT_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ot_threshold",
     .ptr = &s_config.over_temp_threshold,
     .min = 0.0f,
     .max = 200.0f,
     .default_val = FAULT_TEMP_ERROR_C,
     .need_save = true},
    /* === у === */
    {.index = PARAM_SMO_ALPHA,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "smo_alpha",
     .ptr = &motor_data.advanced.smo_alpha,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.1f,
     .need_save = true},
    {.index = PARAM_SMO_BETA,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "smo_beta",
     .ptr = &motor_data.advanced.smo_beta,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.1f,
     .need_save = true},
    {.index = PARAM_FF_FRICTION,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ff_friction",
     .ptr = &motor_data.advanced.ff_friction,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_FW_MAX_CUR,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "fw_max_cur",
     .ptr = &motor_data.advanced.fw_max_current,
     .min = 0.0f,
     .max = 20.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_FW_START_VEL,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "fw_start_vel",
     .ptr = &motor_data.advanced.fw_start_velocity,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = 100.0f,
     .need_save = true},
    {.index = PARAM_COGGING_EN,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "cogging_en",
     .ptr = &motor_data.advanced.cogging_comp_enabled,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_COGGING_CALIB,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_RUNTIME,
     .access = PARAM_ACCESS_RW,
     .name = "cogging_calib",
     .ptr = &motor_data.advanced.cogging_calib_request,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = 0.0f,
     .need_save = false},
    /* === LADRC speed/velocityparam === */
    {.index = PARAM_LADRC_ENABLE,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_en",
     .ptr = &motor_data.ladrc_enable,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = (float)DEFAULT_LADRC_ENABLE,
     .need_save = true},
    {.index = PARAM_LADRC_OMEGA_O,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_wo",
     .ptr = &motor_data.ladrc_config.omega_o,
     .min = 10.0f,
     .max = 5000.0f,
     .default_val = DEFAULT_LADRC_OMEGA_O,
     .need_save = true},
    {.index = PARAM_LADRC_OMEGA_C,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_wc",
     .ptr = &motor_data.ladrc_config.omega_c,
     .min = 5.0f,
     .max = 2000.0f,
     .default_val = DEFAULT_LADRC_OMEGA_C,
     .need_save = true},
    {.index = PARAM_LADRC_B0,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_b0",
     .ptr = &motor_data.ladrc_config.b0,
     .min = 0.1f,
     .max = 10000.0f,
     .default_val = DEFAULT_LADRC_B0,
     .need_save = true},
    {.index = PARAM_LADRC_MAX_OUT,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_max",
     .ptr = &motor_data.ladrc_config.max_output,
     .min = 0.1f,
     .max = 200.0f,
     .default_val = DEFAULT_LADRC_MAX_OUT,
     .need_save = true},
    /* ===================================================================
     * ㄥ�ц?
     * ===================================================================
     *
     * ?:
     *
     * 1.  (0x2000-0x200F): ?
     *    - PARAM_MOTOR_RS, PARAM_MOTOR_LS, PARAM_MOTOR_FLUX,
     * PARAM_MOTOR_POLE_PAIRS
     *
     * 2. PID (0x2010-0x201F): ?
     *    - PARAM_CUR_KP, PARAM_CUR_KI, PARAM_SPD_KP, PARAM_SPD_KI, PARAM_POS_KP
     *
     * 3.  (0x2020-0x202F): ?
     *    - PARAM_LIMIT_TORQUE, PARAM_LIMIT_CURRENT, PARAM_LIMIT_SPEED
     *
     * 4. /″ (0x2030-0x203F): ?
     *    - PARAM_VEL_MAX, PARAM_ACC_SET, PARAM_ACC_RAD, PARAM_INERTIA
     *
     * 5. CAN (0x3000-0x300F): ?
     *    - PARAM_CAN_ID, PARAM_CAN_BAUDRATE, PARAM_PROTOCOL_TYPE,
     * PARAM_CAN_TIMEOUT
     *
     * 6.  (0x3010-0x301F, 0x3030): ?
     *    - PARAM_ZERO_STA, PARAM_ADD_OFFSET, PARAM_DAMPER, PARAM_RUN_MODE
     *
     * 7.  (0x3020-0x302F): ?
     *    - PARAM_OV_THRESHOLD, PARAM_UV_THRESHOLD, PARAM_OC_THRESHOLD,
     * PARAM_OT_THRESHOLD
     *
     * 8. у (0x3040-0x305F): ?
     *    - PARAM_SMO_ALPHA, PARAM_SMO_BETA, PARAM_FF_FRICTION,
     *    - PARAM_FW_MAX_CUR, PARAM_FW_START_VEL, PARAM_COGGING_EN
     *
     * � param_table.h
     * ㄥ
     *
     * ?
     * 1. ?param_table.h ?
     * 2.
     * ㄥāㄥ
     * 3. ㄦㄤ ParamEntry
     * 4. ?param_storage.h ?FlashParamData
     * 
     * 5. ?param_access.c
     * 
     */
};
static const uint32_t s_param_count =
    sizeof(s_param_table) / sizeof(ParamEntry);
/* ============================================================================
 * ㄦｅ?
 * ============================================================================
 */
void ParamTable_Init(void) {
  // ?
  for (uint32_t i = 0; i < s_param_count; i++) {
    const ParamEntry *entry = &s_param_table[i];
    switch (entry->type) {
    case PARAM_TYPE_FLOAT:
      *(float *)entry->ptr = entry->default_val;
      break;
    case PARAM_TYPE_UINT8:
      *(uint8_t *)entry->ptr = (uint8_t)entry->default_val;
      break;
    case PARAM_TYPE_UINT16:
      *(uint16_t *)entry->ptr = (uint16_t)entry->default_val;
      break;
    case PARAM_TYPE_UINT32:
      *(uint32_t *)entry->ptr = (uint32_t)entry->default_val;
      break;
    case PARAM_TYPE_INT32:
      *(int32_t *)entry->ptr = (int32_t)entry->default_val;
      break;
    }
  }
}
const ParamEntry *ParamTable_Find(uint16_t index) {
  // �ф?,ц)
  for (uint32_t i = 0; i < s_param_count; i++) {
    if (s_param_table[i].index == index) {
      return &s_param_table[i];
    }
  }
  return NULL;
}
uint32_t ParamTable_GetCount(void) { return s_param_count; }
const ParamEntry *ParamTable_GetTable(void) { return s_param_table; }
