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
 * @file param_access.c
 * @brief param
 */
#include "param_access.h"
#include "param_encoder_calibration_internal.h"
#include "error_manager.h"
#include "error_types.h"
#include "param_storage.h"
#if !defined(TEST_ENV)
#include "platform.h"
#endif
#include <stdbool.h>
#include <math.h>
#include <string.h>

static volatile bool s_param_save_pending = false;
static volatile uint32_t s_param_save_generation = 0U;
static bool s_param_system_initialized = false;
static bool s_runtime_side_effects_deferred = false;
static FlashParamData s_flash_workspace;
static ParamRuntimeApplyCallback s_runtime_apply_callback = NULL;
static void *s_runtime_apply_context = NULL;

void Param_SetRuntimeApplyCallback(ParamRuntimeApplyCallback callback,
                                   void *context) {
  s_runtime_apply_context = context;
  s_runtime_apply_callback = callback;
}

static void Param_NotifyRuntimeChange(uint16_t index) {
  if (s_runtime_side_effects_deferred || s_runtime_apply_callback == NULL) {
    return;
  }
  s_runtime_apply_callback(s_runtime_apply_context, index);
}

void Param_ApplyRuntimeState(void) {
  Param_NotifyRuntimeChange(PARAM_RUNTIME_APPLY_ALL);
}

static inline bool ParamTable_IsReadable(const ParamEntry *entry) {
  return (entry != NULL) && (entry->access & PARAM_ACCESS_R);
}
/**
 * @brief checkparam
 */
static inline bool ParamTable_IsWritable(const ParamEntry *entry) {
  return (entry != NULL) && (entry->access & PARAM_ACCESS_W);
}
/**
 * @brief checkparam
 */
static bool ParamTable_IsInRange(const ParamEntry *entry, const void *value) {
  float float_val;
  int32_t int_val;
  uint32_t uint_val;

  if (entry == NULL || value == NULL)
    return false;

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    float_val = *(const float *)value;
    return isfinite(float_val) && (float_val >= entry->min) &&
           (float_val <= entry->max);
  case PARAM_TYPE_INT32:
    int_val = *(const int32_t *)value;
    return (int_val >= (int32_t)entry->min) &&
           (int_val <= (int32_t)entry->max);
  case PARAM_TYPE_UINT8:
    uint_val = *(const uint8_t *)value;
    return (uint_val >= (uint32_t)entry->min) &&
           (uint_val <= (uint32_t)entry->max);
  case PARAM_TYPE_UINT16:
    uint_val = *(const uint16_t *)value;
    return (uint_val >= (uint32_t)entry->min) &&
           (uint_val <= (uint32_t)entry->max);
  case PARAM_TYPE_UINT32:
    uint_val = *(const uint32_t *)value;
    return (uint_val >= (uint32_t)entry->min) &&
           (uint_val <= (uint32_t)entry->max);
  default:
    return false;
  }
}

static bool ParamTable_IsDefaultValid(const ParamEntry *entry) {
  if (entry == NULL || !isfinite(entry->default_val)) {
    return false;
  }

  float value = entry->default_val;
  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    return ParamTable_IsInRange(entry, &value);
  case PARAM_TYPE_UINT8: {
    if (value < 0.0f || value > (float)UINT8_MAX) {
      return false;
    }
    uint8_t typed = (uint8_t)value;
    return value == (float)typed && ParamTable_IsInRange(entry, &typed);
  }
  case PARAM_TYPE_UINT16: {
    if (value < 0.0f || value > (float)UINT16_MAX) {
      return false;
    }
    uint16_t typed = (uint16_t)value;
    return value == (float)typed && ParamTable_IsInRange(entry, &typed);
  }
  case PARAM_TYPE_UINT32: {
    if (value < 0.0f || (double)value > (double)UINT32_MAX) {
      return false;
    }
    uint32_t typed = (uint32_t)value;
    return value == (float)typed && ParamTable_IsInRange(entry, &typed);
  }
  case PARAM_TYPE_INT32: {
    if ((double)value < (double)INT32_MIN ||
        (double)value > (double)INT32_MAX) {
      return false;
    }
    int32_t typed = (int32_t)value;
    return value == (float)typed && ParamTable_IsInRange(entry, &typed);
  }
  default:
    return false;
  }
}

static ParamResult Param_CheckWriteType(uint16_t index, ParamType expected,
                                        const char *operation) {
  const ParamEntry *entry = ParamTable_Find(index);
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, operation);
    return PARAM_ERR_INVALID_INDEX;
  }
  if (!ParamTable_IsWritable(entry)) {
    ERROR_REPORT(ERROR_PARAM_ACCESS_DENIED, operation);
    return PARAM_ERR_READONLY;
  }
  if (entry->type != expected) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE, operation);
    return PARAM_ERR_INVALID_TYPE;
  }
  return PARAM_OK;
}

static ParamResult Param_ValidateWriteValue(uint16_t index,
                                            ParamType expected,
                                            const void *value,
                                            const char *operation) {
  ParamResult result = Param_CheckWriteType(index, expected, operation);
  if (result != PARAM_OK) {
    return result;
  }
  const ParamEntry *entry = ParamTable_Find(index);
  if (!ParamTable_IsInRange(entry, value)) {
    ERROR_REPORT(ERROR_PARAM_OUT_OF_RANGE, operation);
    return PARAM_ERR_OUT_OF_RANGE;
  }
  return PARAM_OK;
}
/* ============================================================================
 *
 * ============================================================================
 */
static ParamResult Param_ReadRaw(uint16_t index, void *data,
                                 ParamType expected_type,
                                 const char *operation) {
  if (data == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, operation);
    return PARAM_ERR_NULL_PTR;
  }
  const ParamEntry *entry = ParamTable_Find(index);
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, operation);
    return PARAM_ERR_INVALID_INDEX;
  }
  if (!ParamTable_IsReadable(entry)) {
    ERROR_REPORT(ERROR_PARAM_ACCESS_DENIED, operation);
    return PARAM_ERR_READONLY;
  }
  if (entry->type != expected_type) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE, operation);
    return PARAM_ERR_INVALID_TYPE;
  }
  if (entry->ptr == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, operation);
    return PARAM_ERR_NULL_PTR;
  }

#if !defined(TEST_ENV)
  CRITICAL_SECTION_BEGIN();
#endif
  switch (entry->type) {
  case PARAM_TYPE_UINT8:
    *(uint8_t *)data = *(const uint8_t *)entry->ptr;
    break;
  case PARAM_TYPE_UINT16:
    *(uint16_t *)data = *(const uint16_t *)entry->ptr;
    break;
  case PARAM_TYPE_INT32:
    *(int32_t *)data = *(const int32_t *)entry->ptr;
    break;
  case PARAM_TYPE_UINT32:
    *(uint32_t *)data = *(const uint32_t *)entry->ptr;
    break;
  case PARAM_TYPE_FLOAT:
    *(float *)data = *(const float *)entry->ptr;
    break;
  default:
    break;
  }
#if !defined(TEST_ENV)
  CRITICAL_SECTION_END();
#endif
  return PARAM_OK;
}

static ParamResult Param_WriteRaw(uint16_t index, const void *data) {
  if (data == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_Write: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  const ParamEntry *entry = ParamTable_Find(index);
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, "Param_Write: invalid index");
    return PARAM_ERR_INVALID_INDEX;
  }
  if (!ParamTable_IsWritable(entry)) {
    ERROR_REPORT(ERROR_PARAM_ACCESS_DENIED, "Param_Write: not writable");
    return PARAM_ERR_READONLY;
  }
  if (entry->ptr == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_Write: NULL target");
    return PARAM_ERR_NULL_PTR;
  }
  // check
  if (!ParamTable_IsInRange(entry, data)) {
    ERROR_REPORT(ERROR_PARAM_OUT_OF_RANGE, "Param_Write: out of range");
    return PARAM_ERR_OUT_OF_RANGE;
  }

  ParamResult result = PARAM_OK;
#if !defined(TEST_ENV)
  CRITICAL_SECTION_BEGIN();
#endif
  switch (entry->type) {
  case PARAM_TYPE_UINT8:
    *(uint8_t *)entry->ptr = *(const uint8_t *)data;
    break;
  case PARAM_TYPE_UINT16:
    *(uint16_t *)entry->ptr = *(const uint16_t *)data;
    break;
  case PARAM_TYPE_INT32:
    *(int32_t *)entry->ptr = *(const int32_t *)data;
    break;
  case PARAM_TYPE_UINT32:
    *(uint32_t *)entry->ptr = *(const uint32_t *)data;
    break;
  case PARAM_TYPE_FLOAT:
    *(float *)entry->ptr = *(const float *)data;
    break;
  default:
    result = PARAM_ERR_INVALID_TYPE;
    break;
  }
#if !defined(TEST_ENV)
  CRITICAL_SECTION_END();
#endif
  if (result == PARAM_OK) {
    Param_NotifyRuntimeChange(index);
  }
  return result;
}
/* ============================================================================
 *
 * ============================================================================
 */
ParamResult Param_ReadFloat(uint16_t index, float *value) {
  if (value == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_ReadFloat: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  float tmp = 0.0f;
  ParamResult result = Param_ReadRaw(index, &tmp, PARAM_TYPE_FLOAT,
                                     "Param_ReadFloat: type mismatch");
  if (result != PARAM_OK)
    return result;
  *value = tmp;
  return PARAM_OK;
}
ParamResult Param_WriteFloat(uint16_t index, float value) {
  ParamResult result = Param_CheckWriteType(index, PARAM_TYPE_FLOAT,
                                           "Param_WriteFloat: type mismatch");
  return (result == PARAM_OK) ? Param_WriteRaw(index, &value) : result;
}
ParamResult Param_ReadUint8(uint16_t index, uint8_t *value) {
  if (value == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_ReadUint8: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  uint8_t tmp = 0U;
  ParamResult result = Param_ReadRaw(index, &tmp, PARAM_TYPE_UINT8,
                                     "Param_ReadUint8: type mismatch");
  if (result != PARAM_OK)
    return result;
  *value = tmp;
  return PARAM_OK;
}
ParamResult Param_WriteUint8(uint16_t index, uint8_t value) {
  ParamResult result = Param_CheckWriteType(index, PARAM_TYPE_UINT8,
                                           "Param_WriteUint8: type mismatch");
  return (result == PARAM_OK) ? Param_WriteRaw(index, &value) : result;
}

ParamResult Param_ReadUint16(uint16_t index, uint16_t *value) {
  if (value == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_ReadUint16: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  uint16_t tmp = 0U;
  ParamResult result = Param_ReadRaw(index, &tmp, PARAM_TYPE_UINT16,
                                     "Param_ReadUint16: type mismatch");
  if (result != PARAM_OK) {
    return result;
  }
  *value = tmp;
  return PARAM_OK;
}

ParamResult Param_WriteUint16(uint16_t index, uint16_t value) {
  ParamResult result = Param_CheckWriteType(index, PARAM_TYPE_UINT16,
                                           "Param_WriteUint16: type mismatch");
  return (result == PARAM_OK) ? Param_WriteRaw(index, &value) : result;
}
ParamResult Param_ReadUint32(uint16_t index, uint32_t *value) {
  if (value == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_ReadUint32: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  uint32_t tmp = 0U;
  ParamResult result = Param_ReadRaw(index, &tmp, PARAM_TYPE_UINT32,
                                     "Param_ReadUint32: type mismatch");
  if (result != PARAM_OK)
    return result;
  *value = tmp;
  return PARAM_OK;
}
ParamResult Param_WriteUint32(uint16_t index, uint32_t value) {
  ParamResult result = Param_CheckWriteType(index, PARAM_TYPE_UINT32,
                                           "Param_WriteUint32: type mismatch");
  return (result == PARAM_OK) ? Param_WriteRaw(index, &value) : result;
}

ParamResult Param_ReadInt32(uint16_t index, int32_t *value) {
  if (value == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_ReadInt32: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  int32_t tmp = 0;
  ParamResult result = Param_ReadRaw(index, &tmp, PARAM_TYPE_INT32,
                                     "Param_ReadInt32: type mismatch");
  if (result != PARAM_OK) {
    return result;
  }
  *value = tmp;
  return PARAM_OK;
}

ParamResult Param_WriteInt32(uint16_t index, int32_t value) {
  ParamResult result = Param_CheckWriteType(index, PARAM_TYPE_INT32,
                                           "Param_WriteInt32: type mismatch");
  return (result == PARAM_OK) ? Param_WriteRaw(index, &value) : result;
}

ParamResult Param_ReadAsFloat(uint16_t index, float *value) {
  if (value == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_ReadAsFloat: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  const ParamEntry *entry = ParamTable_Find(index);
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, "Param_ReadAsFloat: invalid index");
    return PARAM_ERR_INVALID_INDEX;
  }

  ParamResult result;
  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    result = Param_ReadFloat(index, value);
    if (result == PARAM_OK && !isfinite(*value)) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    return result;
  case PARAM_TYPE_UINT8: {
    uint8_t typed = 0U;
    result = Param_ReadUint8(index, &typed);
    *value = (float)typed;
    return result;
  }
  case PARAM_TYPE_UINT16: {
    uint16_t typed = 0U;
    result = Param_ReadUint16(index, &typed);
    *value = (float)typed;
    return result;
  }
  case PARAM_TYPE_UINT32: {
    uint32_t typed = 0U;
    result = Param_ReadUint32(index, &typed);
    if (result != PARAM_OK) {
      return result;
    }
    *value = (float)typed;
    if ((double)*value > (double)UINT32_MAX ||
        (uint32_t)*value != typed) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    return PARAM_OK;
  }
  case PARAM_TYPE_INT32: {
    int32_t typed = 0;
    result = Param_ReadInt32(index, &typed);
    if (result != PARAM_OK) {
      return result;
    }
    *value = (float)typed;
    if ((double)*value < (double)INT32_MIN ||
        (double)*value > (double)INT32_MAX || (int32_t)*value != typed) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    return PARAM_OK;
  }
  default:
    return PARAM_ERR_INVALID_TYPE;
  }
}

ParamResult Param_WriteFromFloat(uint16_t index, float value) {
  if (!isfinite(value)) {
    ERROR_REPORT(ERROR_PARAM_OUT_OF_RANGE, "Param_WriteFromFloat: non-finite");
    return PARAM_ERR_OUT_OF_RANGE;
  }
  const ParamEntry *entry = ParamTable_Find(index);
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, "Param_WriteFromFloat: invalid index");
    return PARAM_ERR_INVALID_INDEX;
  }

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    return Param_WriteFloat(index, value);
  case PARAM_TYPE_UINT8:
    if (value < 0.0f || value > (float)UINT8_MAX ||
        value != (float)(uint8_t)value) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    return Param_WriteUint8(index, (uint8_t)value);
  case PARAM_TYPE_UINT16:
    if (value < 0.0f || value > (float)UINT16_MAX ||
        value != (float)(uint16_t)value) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    return Param_WriteUint16(index, (uint16_t)value);
  case PARAM_TYPE_UINT32:
    if (value < 0.0f || (double)value > (double)UINT32_MAX) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    {
      uint32_t typed = (uint32_t)value;
      if (value != (float)typed) {
        return PARAM_ERR_OUT_OF_RANGE;
      }
      return Param_WriteUint32(index, typed);
    }
  case PARAM_TYPE_INT32:
    if ((double)value < (double)INT32_MIN ||
        (double)value > (double)INT32_MAX) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    {
      int32_t typed = (int32_t)value;
      if (value != (float)typed) {
        return PARAM_ERR_OUT_OF_RANGE;
      }
      return Param_WriteInt32(index, typed);
    }
  default:
    return PARAM_ERR_INVALID_TYPE;
  }
}
/* ============================================================================
 *
 * ============================================================================
 */
/**
 * @brief paramparam FlashParamData
 * @note  packed warning
 */
static void CollectParamsToFlashData(FlashParamData *flash_data) {
  memset(flash_data, 0, sizeof(FlashParamData));
  float tmp_float;
  uint8_t tmp_uint8;
  uint32_t tmp_uint32;
  // motorparam
  if (Param_ReadFloat(PARAM_MOTOR_RS, &tmp_float) == PARAM_OK)
    flash_data->motor_rs = tmp_float;
  if (Param_ReadFloat(PARAM_MOTOR_LS, &tmp_float) == PARAM_OK)
    flash_data->motor_ls = tmp_float;
  if (Param_ReadFloat(PARAM_MOTOR_FLUX, &tmp_float) == PARAM_OK)
    flash_data->motor_flux = tmp_float;
  if (Param_ReadUint8(PARAM_MOTOR_POLE_PAIRS, &tmp_uint8) == PARAM_OK)
    flash_data->motor_pole_pairs = tmp_uint8;
  // PIDparam
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
  // PARAM_CUR_FILT_GAIN and PARAM_SPD_FILT_GAIN removed - PidTypeDef has no
  // filter_alpha if (Param_ReadFloat(PARAM_CUR_FILT_GAIN, &tmp_float) ==
  // PARAM_OK)
  //   flash_data->cur_filt_gain = tmp_float;
  // if (Param_ReadFloat(PARAM_SPD_FILT_GAIN, &tmp_float) == PARAM_OK)
  //   flash_data->spd_filt_gain = tmp_float;
  // limitparam
  if (Param_ReadFloat(PARAM_LIMIT_TORQUE, &tmp_float) == PARAM_OK)
    flash_data->limit_torque = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_CURRENT, &tmp_float) == PARAM_OK)
    flash_data->limit_current = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_SPEED, &tmp_float) == PARAM_OK)
    flash_data->limit_speed = tmp_float;
  // position/speed/velocitymodeparam
  if (Param_ReadFloat(PARAM_VEL_MAX, &tmp_float) == PARAM_OK)
    flash_data->vel_max = tmp_float;
  if (Param_ReadFloat(PARAM_ACC_SET, &tmp_float) == PARAM_OK)
    flash_data->acc_set = tmp_float;
  if (Param_ReadFloat(PARAM_ACC_RAD, &tmp_float) == PARAM_OK)
    flash_data->acc_rad = tmp_float;
  if (Param_ReadFloat(PARAM_INERTIA, &tmp_float) == PARAM_OK)
    flash_data->inertia = tmp_float;
  // CANconfig
  if (Param_ReadUint8(PARAM_CAN_ID, &tmp_uint8) == PARAM_OK)
    flash_data->can_id = tmp_uint8;
  if (Param_ReadUint8(PARAM_CAN_BAUDRATE, &tmp_uint8) == PARAM_OK)
    flash_data->can_baudrate = tmp_uint8;
  if (Param_ReadUint8(PARAM_PROTOCOL_TYPE, &tmp_uint8) == PARAM_OK)
    flash_data->protocol_type = tmp_uint8;
  // config
  if (Param_ReadUint8(PARAM_ZERO_STA, &tmp_uint8) == PARAM_OK)
    flash_data->zero_sta = tmp_uint8;
  if (Param_ReadFloat(PARAM_ADD_OFFSET, &tmp_float) == PARAM_OK)
    flash_data->add_offset = tmp_float;
  if (Param_ReadUint8(PARAM_DAMPER, &tmp_uint8) == PARAM_OK)
    flash_data->damper = tmp_uint8;
  if (Param_ReadUint8(PARAM_RUN_MODE, &tmp_uint8) == PARAM_OK)
    flash_data->run_mode = tmp_uint8;
  // CANtimeout ( UINT32 )
  if (Param_ReadUint32(PARAM_CAN_TIMEOUT, &tmp_uint32) == PARAM_OK)
    flash_data->can_timeout = tmp_uint32;
  // protectionconfig
  if (Param_ReadFloat(PARAM_OV_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_voltage_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_UV_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->under_voltage_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_OC_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_current_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_OT_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_temp_threshold = tmp_float;
  // config
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
  ParamEncoderCalibration_Collect(flash_data);
  if (Param_ReadFloat(PARAM_LADRC_ENABLE, &tmp_float) == PARAM_OK)
    flash_data->ladrc_enable = tmp_float;
  if (Param_ReadFloat(PARAM_LADRC_OMEGA_O, &tmp_float) == PARAM_OK)
    flash_data->ladrc_omega_o = tmp_float;
  if (Param_ReadFloat(PARAM_LADRC_OMEGA_C, &tmp_float) == PARAM_OK)
    flash_data->ladrc_omega_c = tmp_float;
  if (Param_ReadFloat(PARAM_LADRC_B0, &tmp_float) == PARAM_OK)
    flash_data->ladrc_b0 = tmp_float;
  if (Param_ReadFloat(PARAM_LADRC_MAX_OUT, &tmp_float) == PARAM_OK)
    flash_data->ladrc_max_output = tmp_float;
  }
/**
 * @brief  FlashParamData param
 * @note  packed
 */
static void ApplyParamsFromFlashData(const FlashParamData *flash_data) {
#if !defined(TEST_ENV)
  CRITICAL_SECTION_BEGIN();
#endif
  s_runtime_side_effects_deferred = true;

  // motorparam
  Param_WriteFloat(PARAM_MOTOR_RS, flash_data->motor_rs);
  Param_WriteFloat(PARAM_MOTOR_LS, flash_data->motor_ls);
  Param_WriteFloat(PARAM_MOTOR_FLUX, flash_data->motor_flux);
  Param_WriteUint8(PARAM_MOTOR_POLE_PAIRS, flash_data->motor_pole_pairs);
  // PIDparam
  Param_WriteFloat(PARAM_CUR_KP, flash_data->cur_kp);
  Param_WriteFloat(PARAM_CUR_KI, flash_data->cur_ki);
  Param_WriteFloat(PARAM_SPD_KP, flash_data->spd_kp);
  Param_WriteFloat(PARAM_SPD_KI, flash_data->spd_ki);
  Param_WriteFloat(PARAM_POS_KP, flash_data->pos_kp);
  // PARAM_CUR_FILT_GAIN and PARAM_SPD_FILT_GAIN removed - PidTypeDef has no
  // filter_alpha Param_WriteFloat(PARAM_CUR_FILT_GAIN,
  // flash_data->cur_filt_gain); Param_WriteFloat(PARAM_SPD_FILT_GAIN,
  // flash_data->spd_filt_gain);
  // limitparam
  Param_WriteFloat(PARAM_LIMIT_TORQUE, flash_data->limit_torque);
  Param_WriteFloat(PARAM_LIMIT_CURRENT, flash_data->limit_current);
  Param_WriteFloat(PARAM_LIMIT_SPEED, flash_data->limit_speed);
  // position/speed/velocitymodeparam
  Param_WriteFloat(PARAM_VEL_MAX, flash_data->vel_max);
  Param_WriteFloat(PARAM_ACC_SET, flash_data->acc_set);
  Param_WriteFloat(PARAM_ACC_RAD, flash_data->acc_rad);
  Param_WriteFloat(PARAM_INERTIA, flash_data->inertia);
  // CANconfig
  Param_WriteUint8(PARAM_CAN_ID, flash_data->can_id);
  Param_WriteUint8(PARAM_CAN_BAUDRATE, flash_data->can_baudrate);
  Param_WriteUint8(PARAM_PROTOCOL_TYPE, flash_data->protocol_type);
  // config
  Param_WriteUint8(PARAM_ZERO_STA, flash_data->zero_sta);
  Param_WriteFloat(PARAM_ADD_OFFSET, flash_data->add_offset);
  Param_WriteUint8(PARAM_DAMPER, flash_data->damper);
  Param_WriteUint8(PARAM_RUN_MODE, flash_data->run_mode);
  // CANtimeout ( UINT32 )
  (void)Param_WriteUint32(PARAM_CAN_TIMEOUT, flash_data->can_timeout);
  // protectionconfig
  Param_WriteFloat(PARAM_OV_THRESHOLD, flash_data->over_voltage_threshold);
  Param_WriteFloat(PARAM_UV_THRESHOLD, flash_data->under_voltage_threshold);
  Param_WriteFloat(PARAM_OC_THRESHOLD, flash_data->over_current_threshold);
  Param_WriteFloat(PARAM_OT_THRESHOLD, flash_data->over_temp_threshold);
  // config
  Param_WriteFloat(PARAM_SMO_ALPHA, flash_data->smo_alpha);
  Param_WriteFloat(PARAM_SMO_BETA, flash_data->smo_beta);
  Param_WriteFloat(PARAM_FF_FRICTION, flash_data->ff_friction);
  Param_WriteFloat(PARAM_FW_MAX_CUR, flash_data->fw_max_current);
  Param_WriteFloat(PARAM_FW_START_VEL, flash_data->fw_start_velocity);
  Param_WriteFloat(PARAM_COGGING_EN, flash_data->cogging_comp_enabled);
  Param_WriteFloat(PARAM_LADRC_ENABLE, flash_data->ladrc_enable);
  Param_WriteFloat(PARAM_LADRC_OMEGA_O, flash_data->ladrc_omega_o);
  Param_WriteFloat(PARAM_LADRC_OMEGA_C, flash_data->ladrc_omega_c);
  Param_WriteFloat(PARAM_LADRC_B0, flash_data->ladrc_b0);
  Param_WriteFloat(PARAM_LADRC_MAX_OUT, flash_data->ladrc_max_output);
  ParamEncoderCalibration_Restore(flash_data);

  s_runtime_side_effects_deferred = false;
#if !defined(TEST_ENV)
  CRITICAL_SECTION_END();
#endif
  Param_ApplyRuntimeState();
}

static ParamResult Param_ValidateFloatValue(uint16_t index, float value) {
  return Param_ValidateWriteValue(index, PARAM_TYPE_FLOAT, &value,
                                  "Flash restore float validation");
}

static ParamResult Param_ValidateUint8Value(uint16_t index, uint8_t value) {
  return Param_ValidateWriteValue(index, PARAM_TYPE_UINT8, &value,
                                  "Flash restore uint8 validation");
}

static ParamResult Param_ValidateUint32Value(uint16_t index, uint32_t value) {
  return Param_ValidateWriteValue(index, PARAM_TYPE_UINT32, &value,
                                  "Flash restore uint32 validation");
}

#define PARAM_RESTORE_CHECK(expression)                                         \
  do {                                                                          \
    ParamResult restore_result = (expression);                                  \
    if (restore_result != PARAM_OK) {                                            \
      return restore_result;                                                     \
    }                                                                           \
  } while (0)

static ParamResult ValidateParamsFromFlashData(
    const FlashParamData *flash_data) {
  if (flash_data == NULL) {
    return PARAM_ERR_NULL_PTR;
  }

  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_MOTOR_RS, flash_data->motor_rs));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_MOTOR_LS, flash_data->motor_ls));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_MOTOR_FLUX, flash_data->motor_flux));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_MOTOR_POLE_PAIRS, flash_data->motor_pole_pairs));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_CUR_KP, flash_data->cur_kp));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_CUR_KI, flash_data->cur_ki));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_SPD_KP, flash_data->spd_kp));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_SPD_KI, flash_data->spd_ki));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_POS_KP, flash_data->pos_kp));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LIMIT_TORQUE, flash_data->limit_torque));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LIMIT_CURRENT, flash_data->limit_current));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LIMIT_SPEED, flash_data->limit_speed));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_VEL_MAX, flash_data->vel_max));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_ACC_SET, flash_data->acc_set));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_ACC_RAD, flash_data->acc_rad));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_INERTIA, flash_data->inertia));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_CAN_ID, flash_data->can_id));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_CAN_BAUDRATE, flash_data->can_baudrate));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_PROTOCOL_TYPE, flash_data->protocol_type));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_ZERO_STA, flash_data->zero_sta));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_ADD_OFFSET, flash_data->add_offset));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_DAMPER, flash_data->damper));
  PARAM_RESTORE_CHECK(Param_ValidateUint8Value(PARAM_RUN_MODE, flash_data->run_mode));
  PARAM_RESTORE_CHECK(Param_ValidateUint32Value(PARAM_CAN_TIMEOUT, flash_data->can_timeout));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_OV_THRESHOLD, flash_data->over_voltage_threshold));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_UV_THRESHOLD, flash_data->under_voltage_threshold));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_OC_THRESHOLD, flash_data->over_current_threshold));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_OT_THRESHOLD, flash_data->over_temp_threshold));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_SMO_ALPHA, flash_data->smo_alpha));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_SMO_BETA, flash_data->smo_beta));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_FF_FRICTION, flash_data->ff_friction));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_FW_MAX_CUR, flash_data->fw_max_current));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_FW_START_VEL, flash_data->fw_start_velocity));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_COGGING_EN, flash_data->cogging_comp_enabled));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LADRC_ENABLE, flash_data->ladrc_enable));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LADRC_OMEGA_O, flash_data->ladrc_omega_o));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LADRC_OMEGA_C, flash_data->ladrc_omega_c));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LADRC_B0, flash_data->ladrc_b0));
  PARAM_RESTORE_CHECK(Param_ValidateFloatValue(PARAM_LADRC_MAX_OUT, flash_data->ladrc_max_output));
  if (!ParamEncoderCalibration_IsFlashDataValid(flash_data)) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE,
                 "Flash restore encoder metadata invalid");
    return PARAM_ERR_OUT_OF_RANGE;
  }

  return PARAM_OK;
}

static ParamResult RestoreParamsFromFlashData(
    const FlashParamData *flash_data) {
  ParamResult result = ValidateParamsFromFlashData(flash_data);
  if (result != PARAM_OK) {
    return result;
  }
  ApplyParamsFromFlashData(flash_data);
  return PARAM_OK;
}

#undef PARAM_RESTORE_CHECK
ParamResult Param_SaveToFlash(void) {
  // param FlashParamData
  CollectParamsToFlashData(&s_flash_workspace);
  ParamResult validation =
      ValidateParamsFromFlashData(&s_flash_workspace);
  if (validation != PARAM_OK) {
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Runtime parameters failed validation");
    return validation;
  }
  //
  FlashStorageResult result = ParamStorage_Save(&s_flash_workspace);
  // error
  switch (result) {
  case FLASH_STORAGE_OK:
    return PARAM_OK;
  case FLASH_STORAGE_ERR_ERASE:
  case FLASH_STORAGE_ERR_WRITE:
  case FLASH_STORAGE_ERR_VERIFY:
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Flash save failed");
    return PARAM_ERR_STORAGE;
  default:
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Flash save error");
    return PARAM_ERR_STORAGE;
  }
}
ParamResult Param_LoadFromFlash(void) {
  //  Flash
  FlashStorageResult result = ParamStorage_Load(&s_flash_workspace);
  if (result != FLASH_STORAGE_OK) {
    ERROR_REPORT(ERROR_PARAM_READ_FAILED, "Flash load failed");
    return PARAM_ERR_STORAGE;
  }
  // param
  ParamResult restore_result =
      RestoreParamsFromFlashData(&s_flash_workspace);
  if (restore_result != PARAM_OK) {
    ERROR_REPORT(ERROR_PARAM_READ_FAILED, "Flash values failed validation");
  }
  return restore_result;
}
ParamResult Param_SystemInitOnce(void) {
  if (s_param_system_initialized) {
    return PARAM_OK;
  }
  ParamTable_Init();
  ParamResult result = Param_LoadFromFlash();
  s_param_system_initialized = true;
  return result;
}
ParamResult Param_RestoreDefaults(void) {
  const ParamEntry *table = ParamTable_GetTable();
  uint32_t count = ParamTable_GetCount();
  if (table == NULL || count == 0) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, "RestoreDefaults: empty table");
    return PARAM_ERR_INVALID_INDEX;
  }

  for (uint32_t i = 0; i < count; i++) {
    const ParamEntry *entry = &table[i];
    if ((entry->access & PARAM_ACCESS_W) &&
        !ParamTable_IsDefaultValid(entry)) {
      ERROR_REPORT(ERROR_PARAM_INVALID_VALUE,
                   "RestoreDefaults: invalid table default");
      return PARAM_ERR_OUT_OF_RANGE;
    }
  }

  ParamResult result = PARAM_OK;
#if !defined(TEST_ENV)
  CRITICAL_SECTION_BEGIN();
#endif
  s_runtime_side_effects_deferred = true;
  for (uint32_t i = 0; i < count; i++) {
    const ParamEntry *entry = &table[i];
    if (!(entry->access & PARAM_ACCESS_W)) {
      continue;
    }

    result = Param_WriteFromFloat(entry->index, entry->default_val);
    if (result != PARAM_OK) {
      ERROR_REPORT(ERROR_PARAM_INVALID_VALUE,
                   "RestoreDefaults: invalid table default");
      break;
    }
  }

  s_runtime_side_effects_deferred = false;
#if !defined(TEST_ENV)
  CRITICAL_SECTION_END();
#endif
  if (result == PARAM_OK) {
    Param_ApplyRuntimeState();
  }
  return result;
}
ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry) {
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_GetInfo: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  *entry = ParamTable_Find(index);
  if (*entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, "Param_GetInfo: invalid index");
    return PARAM_ERR_INVALID_INDEX;
  }
  return PARAM_OK;
}
static void Param_SetSavePending(void) {
#if defined(TEST_ENV)
  s_param_save_pending = true;
#else
  CRITICAL_SECTION_BEGIN();
  s_param_save_pending = true;
  CRITICAL_SECTION_END();
#endif
}

static bool Param_TakeSavePending(void) {
  bool pending;
#if defined(TEST_ENV)
  pending = s_param_save_pending;
  s_param_save_pending = false;
#else
  CRITICAL_SECTION_BEGIN();
  pending = s_param_save_pending;
  s_param_save_pending = false;
  CRITICAL_SECTION_END();
#endif
  return pending;
}

void Param_ScheduleSave(void) {
#if defined(TEST_ENV)
  s_param_save_generation++;
  s_param_save_pending = true;
#else
  CRITICAL_SECTION_BEGIN();
  s_param_save_generation++;
  s_param_save_pending = true;
  CRITICAL_SECTION_END();
#endif
}

bool Param_HasScheduledSave(void) {
  bool pending;
#if defined(TEST_ENV)
  pending = s_param_save_pending;
#else
  CRITICAL_SECTION_BEGIN();
  pending = s_param_save_pending;
  CRITICAL_SECTION_END();
#endif
  return pending;
}

uint32_t Param_GetScheduledSaveGeneration(void) {
  uint32_t generation;
#if defined(TEST_ENV)
  generation = s_param_save_generation;
#else
  CRITICAL_SECTION_BEGIN();
  generation = s_param_save_generation;
  CRITICAL_SECTION_END();
#endif
  return generation;
}

void Param_DiscardScheduledSave(void) {
#if defined(TEST_ENV)
  s_param_save_pending = false;
#else
  CRITICAL_SECTION_BEGIN();
  s_param_save_pending = false;
  CRITICAL_SECTION_END();
#endif
}

bool Param_DiscardScheduledSaveIfGeneration(uint32_t generation) {
  bool discarded = false;
#if defined(TEST_ENV)
  if (s_param_save_generation == generation) {
    s_param_save_pending = false;
    discarded = true;
  }
#else
  CRITICAL_SECTION_BEGIN();
  if (s_param_save_generation == generation) {
    s_param_save_pending = false;
    discarded = true;
  }
  CRITICAL_SECTION_END();
#endif
  return discarded;
}

ParamResult Param_RollbackScheduledSave(void) {
  ParamResult result = Param_LoadFromFlash();
  if (result == PARAM_OK) {
    return PARAM_OK;
  }

  result = Param_RestoreDefaults();
  if (result == PARAM_OK) {
    ParamEncoderCalibration_Clear();
  }
  return result;
}

bool Param_ProcessScheduledSave(void) {
  if (!Param_TakeSavePending()) {
    return false;
  }
  if (Param_SaveToFlash() != PARAM_OK) {
    /* Retry later, while preserving any request raised during this attempt. */
    Param_SetSavePending();
    return false;
  }
  return true;
}
