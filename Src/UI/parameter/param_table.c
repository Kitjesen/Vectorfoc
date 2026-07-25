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
#include <limits.h>
#include <math.h>
#include <stddef.h>
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
     .ptr = NULL,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_MOTOR_LS,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_ls",
     .ptr = NULL,
     .min = 0.0f,
     .max = 0.01f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_MOTOR_FLUX,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_flux",
     .ptr = NULL,
     .min = 0.0f,
     .max = 0.1f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_MOTOR_POLE_PAIRS,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "motor_pole_pairs",
     .ptr = NULL,
     .min = 1,
     .max = 50,
     .default_val = 0.0f,
     .need_save = true},
    /* === PID === */
    {.index = PARAM_CUR_KP,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "cur_kp",
     .ptr = NULL,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_CUR_KI,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "cur_ki",
     .ptr = NULL,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_SPD_KP,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "spd_kp",
     .ptr = NULL,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_SPD_KI,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "spd_ki",
     .ptr = NULL,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_POS_KP, // Same as PARAM_LOC_KP
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "pos_kp",
     .ptr = NULL,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = 0.0f,
     .need_save = true},
    // Note: filter_alpha removed - PidTypeDef doesn't have this member
    // CUR_FILT_GAIN and SPD_FILT_GAIN parameters disabled
    /* ===  === */
    {.index = PARAM_LIMIT_TORQUE,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "limit_torque",
     .ptr = NULL,
     .min = 0.0f,
     .max = 50.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_LIMIT_CURRENT, // Same as PARAM_LIMIT_CUR
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "limit_current",
     .ptr = NULL,
     .min = 0.0f,
     .max = 50.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_LIMIT_SPEED, // Same as PARAM_LIMIT_SPD
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "limit_speed",
     .ptr = NULL,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = 0.0f,
     .need_save = true},
    /* === /″ === */
    {.index = PARAM_VEL_MAX,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "vel_max",
     .ptr = NULL,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_ACC_SET,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "acc_set",
     .ptr = NULL,
     .min = 0.0f,
     .max = 10000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_ACC_RAD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "acc_rad",
     .ptr = NULL,
     .min = 0.0f,
     .max = 10000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_INERTIA,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "inertia",
     .ptr = NULL,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = 0.0f,
     .need_save = true},
    /* === CAN === */
    {.index = PARAM_CAN_ID,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "can_id",
     .ptr = NULL,
     .min = 1,
     .max = 127,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_CAN_BAUDRATE,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "can_baudrate",
     .ptr = NULL,
     .min = 0,
     .max = 2,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_PROTOCOL_TYPE,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "protocol_type",
     .ptr = NULL,
     .min = 0,
     .max = 2,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_CAN_TIMEOUT,
     .type = PARAM_TYPE_UINT32,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "can_timeout",
     .ptr = NULL,
     .min = 0,
     .max = 10000,
     .default_val = 0.0f,
     .need_save = true},
    /* ===  === */
    {.index = PARAM_ZERO_STA,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "zero_sta",
     .ptr = NULL,
     .min = 0,
     .max = 1,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_ADD_OFFSET,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "add_offset",
     .ptr = NULL,
     .min = -6.28f,
     .max = 6.28f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_DAMPER,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "damper",
     .ptr = NULL,
     .min = 0,
     .max = 1,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_RUN_MODE,
     .type = PARAM_TYPE_UINT8,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "run_mode",
     .ptr = NULL,
     .min = 0,
     .max = 5,
     .default_val = 0.0f,
     .need_save = true},
    /* ===  === */
    {.index = PARAM_OV_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ov_threshold",
     .ptr = NULL,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_UV_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "uv_threshold",
     .ptr = NULL,
     .min = 0.0f,
     .max = 100.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_OC_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "oc_threshold",
     .ptr = NULL,
     .min = 0.0f,
     .max = 200.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_OT_THRESHOLD,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ot_threshold",
     .ptr = NULL,
     .min = 0.0f,
     .max = 200.0f,
     .default_val = 0.0f,
     .need_save = true},
    /* === у === */
    {.index = PARAM_SMO_ALPHA,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "smo_alpha",
     .ptr = NULL,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_SMO_BETA,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "smo_beta",
     .ptr = NULL,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_FF_FRICTION,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ff_friction",
     .ptr = NULL,
     .min = 0.0f,
     .max = 10.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_FW_MAX_CUR,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "fw_max_cur",
     .ptr = NULL,
     .min = 0.0f,
     .max = 20.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_FW_START_VEL,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "fw_start_vel",
     .ptr = NULL,
     .min = 0.0f,
     .max = 1000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_COGGING_EN,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "cogging_en",
     .ptr = NULL,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_COGGING_CALIB,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_RUNTIME,
     .access = PARAM_ACCESS_RW,
     .name = "cogging_calib",
     .ptr = NULL,
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
     .ptr = NULL,
     .min = 0.0f,
     .max = 1.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_LADRC_OMEGA_O,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_wo",
     .ptr = NULL,
     .min = 10.0f,
     .max = 5000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_LADRC_OMEGA_C,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_wc",
     .ptr = NULL,
     .min = 5.0f,
     .max = 2000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_LADRC_B0,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_b0",
     .ptr = NULL,
     .min = 0.1f,
     .max = 10000.0f,
     .default_val = 0.0f,
     .need_save = true},
    {.index = PARAM_LADRC_MAX_OUT,
     .type = PARAM_TYPE_FLOAT,
     .attr = PARAM_ATTR_PERSISTENT,
     .access = PARAM_ACCESS_RW,
     .name = "ladrc_max",
     .ptr = NULL,
     .min = 0.1f,
     .max = 200.0f,
     .default_val = 0.0f,
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
    sizeof(s_param_table) / sizeof(s_param_table[0]);
static ParamTargetBindingAdapter s_binding_adapter = NULL;
static void *s_binding_context = NULL;

static bool ParamTable_IsBindingDefaultValid(const ParamEntry *entry,
                                             float default_val) {
  if (!isfinite(default_val) || default_val < entry->min ||
      default_val > entry->max) {
    return false;
  }

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    return true;
  case PARAM_TYPE_UINT8: {
    if (default_val < 0.0f || default_val > (float)UINT8_MAX) {
      return false;
    }
    uint8_t typed = (uint8_t)default_val;
    return default_val == (float)typed;
  }
  case PARAM_TYPE_UINT16: {
    if (default_val < 0.0f || default_val > (float)UINT16_MAX) {
      return false;
    }
    uint16_t typed = (uint16_t)default_val;
    return default_val == (float)typed;
  }
  case PARAM_TYPE_UINT32: {
    if (default_val < 0.0f || (double)default_val > (double)UINT32_MAX) {
      return false;
    }
    uint32_t typed = (uint32_t)default_val;
    return default_val == (float)typed;
  }
  case PARAM_TYPE_INT32: {
    if ((double)default_val < (double)INT32_MIN ||
        (double)default_val > (double)INT32_MAX) {
      return false;
    }
    int32_t typed = (int32_t)default_val;
    return default_val == (float)typed;
  }
  default:
    return false;
  }
}

static ParamResult ParamTable_ValidateBinding(const ParamEntry *entry,
                                              const ParamTargetBinding *binding) {
  if (binding->index != entry->index) {
    return PARAM_ERR_INVALID_INDEX;
  }
  if (binding->type != entry->type) {
    return PARAM_ERR_INVALID_TYPE;
  }
  if (binding->target == NULL) {
    return PARAM_ERR_NULL_PTR;
  }
  if (!ParamTable_IsBindingDefaultValid(entry, binding->default_val)) {
    return PARAM_ERR_OUT_OF_RANGE;
  }
  return PARAM_OK;
}

ParamResult ParamTable_SetBindingAdapter(ParamTargetBindingAdapter adapter,
                                         void *context) {
  if (adapter == NULL) {
    return PARAM_ERR_NULL_PTR;
  }

  for (uint32_t i = 0U; i < s_param_count; ++i) {
    const ParamEntry *entry = &s_param_table[i];
    ParamTargetBinding binding = {0};
    if (!adapter(context, entry->index, entry->type, &binding)) {
      return PARAM_ERR_INVALID_INDEX;
    }
    ParamResult result = ParamTable_ValidateBinding(entry, &binding);
    if (result != PARAM_OK) {
      return result;
    }
  }

  s_binding_adapter = adapter;
  s_binding_context = context;
  return PARAM_OK;
}

bool ParamTable_IsBound(void) { return s_binding_adapter != NULL; }

ParamResult ParamTable_GetBinding(const ParamEntry *entry,
                                  ParamTargetBinding *binding) {
  if (entry == NULL || binding == NULL || s_binding_adapter == NULL) {
    return PARAM_ERR_NULL_PTR;
  }

  if (!s_binding_adapter(s_binding_context, entry->index, entry->type,
                         binding)) {
    return PARAM_ERR_INVALID_INDEX;
  }
  return ParamTable_ValidateBinding(entry, binding);
}

/* ============================================================================
 * ㄦｅ?
 * ============================================================================
 */
void ParamTable_Init(void) {
  if (s_binding_adapter == NULL) {
    return;
  }
  // ?
  for (uint32_t i = 0; i < s_param_count; i++) {
    const ParamEntry *entry = &s_param_table[i];
    ParamTargetBinding binding = {0};
    if (ParamTable_GetBinding(entry, &binding) != PARAM_OK) {
      return;
    }
    switch (entry->type) {
    case PARAM_TYPE_FLOAT:
      *(float *)binding.target = binding.default_val;
      break;
    case PARAM_TYPE_UINT8:
      *(uint8_t *)binding.target = (uint8_t)binding.default_val;
      break;
    case PARAM_TYPE_UINT16:
      *(uint16_t *)binding.target = (uint16_t)binding.default_val;
      break;
    case PARAM_TYPE_UINT32:
      *(uint32_t *)binding.target = (uint32_t)binding.default_val;
      break;
    case PARAM_TYPE_INT32:
      *(int32_t *)binding.target = (int32_t)binding.default_val;
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
