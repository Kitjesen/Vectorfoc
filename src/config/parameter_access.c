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
 * @file parameter_access.c
 * @brief param
 */
#include "parameter_access.h"
#include "error_manager.h"
#include "error_types.h"
#include "parameter_storage.h"
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
static volatile bool s_param_save_pending = false;
static bool s_param_system_initialized = false;
/* ============================================================================
 *
 * ============================================================================
 */
/**
 * @brief checkparam
 */
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
  if (entry == NULL || value == NULL)
    return false;
  switch (entry->type) {
  case PARAM_TYPE_FLOAT: {
    float val = *(const float *)value;
    return (val >= entry->min) && (val <= entry->max);
  }
  case PARAM_TYPE_INT32: {
    int32_t val = *(const int32_t *)value;
    return (val >= (int32_t)entry->min) && (val <= (int32_t)entry->max);
  }
  case PARAM_TYPE_UINT8:
  case PARAM_TYPE_UINT16:
  case PARAM_TYPE_UINT32: {
    // ，
    uint32_t val = 0;
    if (entry->type == PARAM_TYPE_UINT8)
      val = *(const uint8_t *)value;
    else if (entry->type == PARAM_TYPE_UINT16)
      val = *(const uint16_t *)value;
    else
      val = *(const uint32_t *)value;
    return (val >= (uint32_t)entry->min) && (val <= (uint32_t)entry->max);
  }
  default:
    return false;
  }
}
/* ============================================================================
 *
 * ============================================================================
 */
ParamResult Param_Read(uint16_t index, void *data, ParamType *type) {
  if (data == NULL) {
    ERROR_REPORT(ERROR_PARAM_NULL_PTR, "Param_Read: NULL pointer");
    return PARAM_ERR_NULL_PTR;
  }
  const ParamEntry *entry = ParamTable_Find(index);
  if (entry == NULL) {
    ERROR_REPORT(ERROR_PARAM_INVALID_INDEX, "Param_Read: invalid index");
    return PARAM_ERR_INVALID_INDEX;
  }
  if (!ParamTable_IsReadable(entry)) {
    ERROR_REPORT(ERROR_PARAM_ACCESS_DENIED, "Param_Read: not readable");
    return PARAM_ERR_READONLY;
  }
  // ，
  if (type != NULL) {
    *type = entry->type;
  }
  //
  switch (entry->type) {
  case PARAM_TYPE_UINT8:
    *(uint8_t *)data = *(uint8_t *)entry->ptr;
    break;
  case PARAM_TYPE_UINT16:
    *(uint16_t *)data = *(uint16_t *)entry->ptr;
    break;
  case PARAM_TYPE_INT32:
    *(int32_t *)data = *(int32_t *)entry->ptr;
    break;
  case PARAM_TYPE_UINT32:
    *(uint32_t *)data = *(uint32_t *)entry->ptr;
    break;
  case PARAM_TYPE_FLOAT:
    *(float *)data = *(float *)entry->ptr;
    break;
  default:
    return PARAM_ERR_INVALID_TYPE;
  }
  return PARAM_OK;
}
ParamResult Param_Write(uint16_t index, const void *data) {
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
  // check
  if (!ParamTable_IsInRange(entry, data)) {
    ERROR_REPORT(ERROR_PARAM_OUT_OF_RANGE, "Param_Write: out of range");
    return PARAM_ERR_OUT_OF_RANGE;
  }
  //
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
    return PARAM_ERR_INVALID_TYPE;
  }
  return PARAM_OK;
}
/* ============================================================================
 *
 * ============================================================================
 */
ParamResult Param_ReadFloat(uint16_t index, float *value) {
  ParamType type;
  ParamResult result = Param_Read(index, value, &type);
  if (result != PARAM_OK)
    return result;
  if (type != PARAM_TYPE_FLOAT) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE, "Param_ReadFloat: type mismatch");
    return PARAM_ERR_INVALID_TYPE;
  }
  return PARAM_OK;
}
ParamResult Param_WriteFloat(uint16_t index, float value) {
  return Param_Write(index, &value);
}
ParamResult Param_ReadUint8(uint16_t index, uint8_t *value) {
  ParamType type;
  ParamResult result = Param_Read(index, value, &type);
  if (result != PARAM_OK)
    return result;
  if (type != PARAM_TYPE_UINT8) {
    ERROR_REPORT(ERROR_PARAM_INVALID_VALUE, "Param_ReadUint8: type mismatch");
    return PARAM_ERR_INVALID_TYPE;
  }
  return PARAM_OK;
}
ParamResult Param_WriteUint8(uint16_t index, uint8_t value) {
  return Param_Write(index, &value);
}
/* ============================================================================
 *
 * ============================================================================
 */
/* The saved field set is intentionally explicit: LADRC and runtime-only
 * parameters are not part of the existing Flash image. Keep this order and the
 * packed FlashParamData layout independent of parameter-table attributes. */
typedef struct {
  uint16_t index;
  uint16_t offset;
  ParamType type;
} FlashParamBinding;
#define FLASH_FIELD(index, member, type) \
  {index, offsetof(FlashParamData, member), type}
static const FlashParamBinding s_flash_bindings[] = {
  FLASH_FIELD(PARAM_MOTOR_RS, motor_rs, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_MOTOR_LS, motor_ls, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_MOTOR_FLUX, motor_flux, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_MOTOR_POLE_PAIRS, motor_pole_pairs, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_CUR_KP, cur_kp, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_CUR_KI, cur_ki, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_SPD_KP, spd_kp, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_SPD_KI, spd_ki, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_POS_KP, pos_kp, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_LIMIT_TORQUE, limit_torque, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_LIMIT_CURRENT, limit_current, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_LIMIT_SPEED, limit_speed, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_VEL_MAX, vel_max, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_ACC_SET, acc_set, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_ACC_RAD, acc_rad, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_INERTIA, inertia, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_CAN_ID, can_id, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_CAN_BAUDRATE, can_baudrate, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_PROTOCOL_TYPE, protocol_type, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_ZERO_STA, zero_sta, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_ADD_OFFSET, add_offset, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_DAMPER, damper, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_RUN_MODE, run_mode, PARAM_TYPE_UINT8),
  FLASH_FIELD(PARAM_CAN_TIMEOUT, can_timeout, PARAM_TYPE_UINT32),
  FLASH_FIELD(PARAM_OV_THRESHOLD, over_voltage_threshold, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_UV_THRESHOLD, under_voltage_threshold, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_OC_THRESHOLD, over_current_threshold, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_OT_THRESHOLD, over_temp_threshold, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_SMO_ALPHA, smo_alpha, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_SMO_BETA, smo_beta, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_FF_FRICTION, ff_friction, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_FW_MAX_CUR, fw_max_current, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_FW_START_VEL, fw_start_velocity, PARAM_TYPE_FLOAT),
  FLASH_FIELD(PARAM_COGGING_EN, cogging_comp_enabled, PARAM_TYPE_FLOAT),
};
#undef FLASH_FIELD
#define FLASH_BINDING_COUNT (sizeof(s_flash_bindings) / sizeof(s_flash_bindings[0]))

/* CAN timeout historically bypasses access/range checks during Flash transfer.
 * Preserve that exception without changing the public typed access functions. */
static uint32_t *FlashTimeoutValue(uint16_t index) {
  const ParamEntry *entry = NULL;
  if (Param_GetInfo(index, &entry) == PARAM_OK && entry != NULL &&
      entry->type == PARAM_TYPE_UINT32) return entry->ptr;
  return NULL;
}
static void CollectParamsToFlashData(FlashParamData *flash_data) {
  memset(flash_data, 0, sizeof(*flash_data));
  for (size_t i = 0; i < FLASH_BINDING_COUNT; ++i) {
    const FlashParamBinding *field = &s_flash_bindings[i];
    uint8_t *dest = (uint8_t *)flash_data + field->offset;
    if (field->type == PARAM_TYPE_FLOAT) {
      float value;
      if (Param_ReadFloat(field->index, &value) == PARAM_OK)
        memcpy(dest, &value, sizeof(value));
    } else if (field->type == PARAM_TYPE_UINT8) {
      uint8_t value;
      if (Param_ReadUint8(field->index, &value) == PARAM_OK)
        memcpy(dest, &value, sizeof(value));
    } else {
      const uint32_t *value = FlashTimeoutValue(field->index);
      if (value) memcpy(dest, value, sizeof(*value));
    }
  }
}
static void RestoreParamsFromFlashData(const FlashParamData *flash_data) {
  for (size_t i = 0; i < FLASH_BINDING_COUNT; ++i) {
    const FlashParamBinding *field = &s_flash_bindings[i];
    const uint8_t *src = (const uint8_t *)flash_data + field->offset;
    if (field->type == PARAM_TYPE_FLOAT) {
      float value;
      memcpy(&value, src, sizeof(value));
      Param_WriteFloat(field->index, value);
    } else if (field->type == PARAM_TYPE_UINT8) {
      Param_WriteUint8(field->index, *src);
    } else {
      uint32_t *value = FlashTimeoutValue(field->index);
      if (value) memcpy(value, src, sizeof(*value));
    }
  }
}
ParamResult Param_SaveToFlash(void) {
  FlashParamData flash_data;
  // param FlashParamData
  CollectParamsToFlashData(&flash_data);
  //
  FlashStorageResult result = ParamStorage_Save(&flash_data);
  // error
  switch (result) {
  case FLASH_STORAGE_OK:
    return PARAM_OK;
  case FLASH_STORAGE_ERR_ERASE:
  case FLASH_STORAGE_ERR_WRITE:
  case FLASH_STORAGE_ERR_VERIFY:
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Flash save failed");
    return PARAM_ERR_INVALID_TYPE; // error
  default:
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Flash save error");
    return PARAM_ERR_INVALID_TYPE;
  }
}
ParamResult Param_LoadFromFlash(void) {
  // check Flash
  if (!ParamStorage_HasValidData()) {
    ERROR_REPORT(ERROR_PARAM_READ_FAILED, "No valid Flash data");
    return PARAM_ERR_INVALID_INDEX;
  }
  FlashParamData flash_data;
  //  Flash
  FlashStorageResult result = ParamStorage_Load(&flash_data);
  if (result != FLASH_STORAGE_OK) {
    ERROR_REPORT(ERROR_PARAM_READ_FAILED, "Flash load failed");
    return PARAM_ERR_INVALID_TYPE;
  }
  // param
  RestoreParamsFromFlashData(&flash_data);
  return PARAM_OK;
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
  // param，param
  for (uint32_t i = 0; i < count; i++) {
    const ParamEntry *entry = &table[i];
    // param
    if (!(entry->access & PARAM_ACCESS_W)) {
      continue;
    }
    //
    switch (entry->type) {
    case PARAM_TYPE_UINT8: {
      uint8_t val = (uint8_t)entry->default_val;
      *(uint8_t *)entry->ptr = val;
      break;
    }
    case PARAM_TYPE_UINT16: {
      uint16_t val = (uint16_t)entry->default_val;
      *(uint16_t *)entry->ptr = val;
      break;
    }
    case PARAM_TYPE_INT32: {
      int32_t val = (int32_t)entry->default_val;
      *(int32_t *)entry->ptr = val;
      break;
    }
    case PARAM_TYPE_UINT32: {
      uint32_t val = (uint32_t)entry->default_val;
      *(uint32_t *)entry->ptr = val;
      break;
    }
    case PARAM_TYPE_FLOAT: {
      *(float *)entry->ptr = entry->default_val;
      break;
    }
    default:
      break;
    }
  }
  return PARAM_OK;
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
void Param_ScheduleSave(void) { s_param_save_pending = true; }
bool Param_ProcessScheduledSave(void) {
  if (s_param_save_pending) {
    s_param_save_pending = false;
    Param_SaveToFlash();
    return true;
  }
  return false;
}
