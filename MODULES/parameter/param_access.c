/**
 * @file param_access.c
 * @brief 参数访问实现
 */

#include "param_access.h"
#include "error_manager.h"
#include "error_types.h"
#include "param_storage.h"
#include <string.h>

#include <stdbool.h>

static volatile bool s_param_save_pending = false;
static bool s_param_system_initialized = false;

/* ============================================================================
 * 私有辅助函数
 * ============================================================================
 */

/**
 * @brief 检查参数是否可读
 */
static inline bool ParamTable_IsReadable(const ParamEntry *entry) {
  return (entry != NULL) && (entry->access & PARAM_ACCESS_R);
}

/**
 * @brief 检查参数是否可写
 */
static inline bool ParamTable_IsWritable(const ParamEntry *entry) {
  return (entry != NULL) && (entry->access & PARAM_ACCESS_W);
}

/**
 * @brief 检查参数值是否在有效范围内
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
    // 对于无符号整数，直接比较
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
 * 公共接口实现
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

  // 如果调用者需要类型信息，则返回
  if (type != NULL) {
    *type = entry->type;
  }

  // 根据类型复制数据
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

  // 范围检查
  if (!ParamTable_IsInRange(entry, data)) {
    ERROR_REPORT(ERROR_PARAM_OUT_OF_RANGE, "Param_Write: out of range");
    return PARAM_ERR_OUT_OF_RANGE;
  }

  // 根据类型写入数据
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
 * 便捷接口实现
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
 * 持久化接口实现
 * ============================================================================
 */

/**
 * @brief 从参数表收集所有可持久化参数到 FlashParamData 结构
 * @note 使用临时变量避免 packed 结构体对齐警告
 */
static void CollectParamsToFlashData(FlashParamData *flash_data) {
  memset(flash_data, 0, sizeof(FlashParamData));

  float tmp_float;
  uint8_t tmp_uint8;

  // 电机参数
  if (Param_ReadFloat(PARAM_MOTOR_RS, &tmp_float) == PARAM_OK)
    flash_data->motor_rs = tmp_float;
  if (Param_ReadFloat(PARAM_MOTOR_LS, &tmp_float) == PARAM_OK)
    flash_data->motor_ls = tmp_float;
  if (Param_ReadFloat(PARAM_MOTOR_FLUX, &tmp_float) == PARAM_OK)
    flash_data->motor_flux = tmp_float;
  if (Param_ReadUint8(PARAM_MOTOR_POLE_PAIRS, &tmp_uint8) == PARAM_OK)
    flash_data->motor_pole_pairs = tmp_uint8;

  // PID参数
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

  // 限制参数
  if (Param_ReadFloat(PARAM_LIMIT_TORQUE, &tmp_float) == PARAM_OK)
    flash_data->limit_torque = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_CURRENT, &tmp_float) == PARAM_OK)
    flash_data->limit_current = tmp_float;
  if (Param_ReadFloat(PARAM_LIMIT_SPEED, &tmp_float) == PARAM_OK)
    flash_data->limit_speed = tmp_float;

  // 位置/速度模式参数
  if (Param_ReadFloat(PARAM_VEL_MAX, &tmp_float) == PARAM_OK)
    flash_data->vel_max = tmp_float;
  if (Param_ReadFloat(PARAM_ACC_SET, &tmp_float) == PARAM_OK)
    flash_data->acc_set = tmp_float;
  if (Param_ReadFloat(PARAM_ACC_RAD, &tmp_float) == PARAM_OK)
    flash_data->acc_rad = tmp_float;
  if (Param_ReadFloat(PARAM_INERTIA, &tmp_float) == PARAM_OK)
    flash_data->inertia = tmp_float;

  // CAN配置
  if (Param_ReadUint8(PARAM_CAN_ID, &tmp_uint8) == PARAM_OK)
    flash_data->can_id = tmp_uint8;
  if (Param_ReadUint8(PARAM_CAN_BAUDRATE, &tmp_uint8) == PARAM_OK)
    flash_data->can_baudrate = tmp_uint8;
  if (Param_ReadUint8(PARAM_PROTOCOL_TYPE, &tmp_uint8) == PARAM_OK)
    flash_data->protocol_type = tmp_uint8;

  // 功能配置
  if (Param_ReadUint8(PARAM_ZERO_STA, &tmp_uint8) == PARAM_OK)
    flash_data->zero_sta = tmp_uint8;
  if (Param_ReadFloat(PARAM_ADD_OFFSET, &tmp_float) == PARAM_OK)
    flash_data->add_offset = tmp_float;
  if (Param_ReadUint8(PARAM_DAMPER, &tmp_uint8) == PARAM_OK)
    flash_data->damper = tmp_uint8;
  if (Param_ReadUint8(PARAM_RUN_MODE, &tmp_uint8) == PARAM_OK)
    flash_data->run_mode = tmp_uint8;

  // CAN超时 (特殊处理 UINT32 类型)
  const ParamEntry *entry = NULL;
  if (Param_GetInfo(PARAM_CAN_TIMEOUT, &entry) == PARAM_OK && entry != NULL) {
    if (entry->type == PARAM_TYPE_UINT32) {
      flash_data->can_timeout = *(uint32_t *)entry->ptr;
    }
  }

  // 保护配置
  if (Param_ReadFloat(PARAM_OV_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_voltage_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_UV_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->under_voltage_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_OC_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_current_threshold = tmp_float;
  if (Param_ReadFloat(PARAM_OT_THRESHOLD, &tmp_float) == PARAM_OK)
    flash_data->over_temp_threshold = tmp_float;

  // 高级控制配置
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

/**
 * @brief 从 FlashParamData 结构恢复参数到内存
 * @note 直接赋值避免 packed 结构体对齐问题
 */
static void RestoreParamsFromFlashData(const FlashParamData *flash_data) {
  // 电机参数
  Param_WriteFloat(PARAM_MOTOR_RS, flash_data->motor_rs);
  Param_WriteFloat(PARAM_MOTOR_LS, flash_data->motor_ls);
  Param_WriteFloat(PARAM_MOTOR_FLUX, flash_data->motor_flux);
  Param_WriteUint8(PARAM_MOTOR_POLE_PAIRS, flash_data->motor_pole_pairs);

  // PID参数
  Param_WriteFloat(PARAM_CUR_KP, flash_data->cur_kp);
  Param_WriteFloat(PARAM_CUR_KI, flash_data->cur_ki);
  Param_WriteFloat(PARAM_SPD_KP, flash_data->spd_kp);
  Param_WriteFloat(PARAM_SPD_KI, flash_data->spd_ki);
  Param_WriteFloat(PARAM_POS_KP, flash_data->pos_kp);
  // PARAM_CUR_FILT_GAIN and PARAM_SPD_FILT_GAIN removed - PidTypeDef has no
  // filter_alpha Param_WriteFloat(PARAM_CUR_FILT_GAIN,
  // flash_data->cur_filt_gain); Param_WriteFloat(PARAM_SPD_FILT_GAIN,
  // flash_data->spd_filt_gain);

  // 限制参数
  Param_WriteFloat(PARAM_LIMIT_TORQUE, flash_data->limit_torque);
  Param_WriteFloat(PARAM_LIMIT_CURRENT, flash_data->limit_current);
  Param_WriteFloat(PARAM_LIMIT_SPEED, flash_data->limit_speed);

  // 位置/速度模式参数
  Param_WriteFloat(PARAM_VEL_MAX, flash_data->vel_max);
  Param_WriteFloat(PARAM_ACC_SET, flash_data->acc_set);
  Param_WriteFloat(PARAM_ACC_RAD, flash_data->acc_rad);
  Param_WriteFloat(PARAM_INERTIA, flash_data->inertia);

  // CAN配置
  Param_WriteUint8(PARAM_CAN_ID, flash_data->can_id);
  Param_WriteUint8(PARAM_CAN_BAUDRATE, flash_data->can_baudrate);
  Param_WriteUint8(PARAM_PROTOCOL_TYPE, flash_data->protocol_type);

  // 功能配置
  Param_WriteUint8(PARAM_ZERO_STA, flash_data->zero_sta);
  Param_WriteFloat(PARAM_ADD_OFFSET, flash_data->add_offset);
  Param_WriteUint8(PARAM_DAMPER, flash_data->damper);
  Param_WriteUint8(PARAM_RUN_MODE, flash_data->run_mode);

  // CAN超时 (特殊处理 UINT32 类型)
  const ParamEntry *entry = NULL;
  if (Param_GetInfo(PARAM_CAN_TIMEOUT, &entry) == PARAM_OK && entry != NULL) {
    if (entry->type == PARAM_TYPE_UINT32) {
      *(uint32_t *)entry->ptr = flash_data->can_timeout;
    }
  }

  // 保护配置
  Param_WriteFloat(PARAM_OV_THRESHOLD, flash_data->over_voltage_threshold);
  Param_WriteFloat(PARAM_UV_THRESHOLD, flash_data->under_voltage_threshold);
  Param_WriteFloat(PARAM_OC_THRESHOLD, flash_data->over_current_threshold);
  Param_WriteFloat(PARAM_OT_THRESHOLD, flash_data->over_temp_threshold);

  // 高级控制配置
  Param_WriteFloat(PARAM_SMO_ALPHA, flash_data->smo_alpha);
  Param_WriteFloat(PARAM_SMO_BETA, flash_data->smo_beta);
  Param_WriteFloat(PARAM_FF_FRICTION, flash_data->ff_friction);
  Param_WriteFloat(PARAM_FW_MAX_CUR, flash_data->fw_max_current);
  Param_WriteFloat(PARAM_FW_START_VEL, flash_data->fw_start_velocity);
  Param_WriteFloat(PARAM_COGGING_EN, flash_data->cogging_comp_enabled);
}

ParamResult Param_SaveToFlash(void) {
  FlashParamData flash_data;

  // 收集所有参数到 FlashParamData 结构
  CollectParamsToFlashData(&flash_data);

  // 调用底层存储接口保存
  FlashStorageResult result = ParamStorage_Save(&flash_data);

  // 转换错误码
  switch (result) {
  case FLASH_STORAGE_OK:
    return PARAM_OK;
  case FLASH_STORAGE_ERR_ERASE:
  case FLASH_STORAGE_ERR_WRITE:
  case FLASH_STORAGE_ERR_VERIFY:
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Flash save failed");
    return PARAM_ERR_INVALID_TYPE; // 使用现有错误码
  default:
    ERROR_REPORT(ERROR_PARAM_WRITE_FAILED, "Flash save error");
    return PARAM_ERR_INVALID_TYPE;
  }
}

ParamResult Param_LoadFromFlash(void) {
  // 首先检查 Flash 中是否有有效数据
  if (!ParamStorage_HasValidData()) {
    ERROR_REPORT(ERROR_PARAM_READ_FAILED, "No valid Flash data");
    return PARAM_ERR_INVALID_INDEX;
  }

  FlashParamData flash_data;

  // 从 Flash 加载数据
  FlashStorageResult result = ParamStorage_Load(&flash_data);

  if (result != FLASH_STORAGE_OK) {
    ERROR_REPORT(ERROR_PARAM_READ_FAILED, "Flash load failed");
    return PARAM_ERR_INVALID_TYPE;
  }

  // 将加载的数据恢复到内存中的参数
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

  // 遍历参数表，将每个参数恢复为默认值
  for (uint32_t i = 0; i < count; i++) {
    const ParamEntry *entry = &table[i];

    // 跳过只读参数
    if (!(entry->access & PARAM_ACCESS_W)) {
      continue;
    }

    // 根据类型写入默认值
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
