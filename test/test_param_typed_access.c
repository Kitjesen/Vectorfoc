// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "param_access.h"
#include "param_storage.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_PARAM_UINT16 0x7FFEU
#define TEST_PARAM_INT32 0x7FFFU

static float s_float_value = 2.5f;
static float s_float_value_2 = 1.25f;
static uint8_t s_uint8_value = 7U;
static uint8_t s_uint8_value_2 = 1U;
static uint16_t s_uint16_value = 100U;
static uint32_t s_uint32_value = 1000U;
static int32_t s_int32_value = -1000;
static int s_error_count;
static FlashStorageResult s_storage_result = FLASH_STORAGE_OK;
static int s_storage_save_count;
static FlashParamData s_storage_image;

static const ParamEntry s_entries[] = {
    {.index = PARAM_MOTOR_RS,
     .type = PARAM_TYPE_FLOAT,
     .access = PARAM_ACCESS_RW,
     .ptr = &s_float_value,
     .min = 0.0f,
     .max = 10.0f},
    {.index = PARAM_CAN_ID,
     .type = PARAM_TYPE_UINT8,
     .access = PARAM_ACCESS_RW,
     .ptr = &s_uint8_value,
     .min = 1.0f,
     .max = 127.0f},
    {.index = PARAM_CAN_TIMEOUT,
     .type = PARAM_TYPE_UINT32,
     .access = PARAM_ACCESS_RW,
     .ptr = &s_uint32_value,
     .min = 0.0f,
     .max = 10000.0f},
    {.index = TEST_PARAM_UINT16,
     .type = PARAM_TYPE_UINT16,
     .access = PARAM_ACCESS_RW,
     .ptr = &s_uint16_value,
     .min = 0.0f,
     .max = 1000.0f},
    {.index = TEST_PARAM_INT32,
     .type = PARAM_TYPE_INT32,
     .access = PARAM_ACCESS_RW,
     .ptr = &s_int32_value,
     .min = -2000.0f,
     .max = 2000.0f},
};

static const ParamIndex s_restore_float_indices[] = {
    PARAM_MOTOR_LS,      PARAM_MOTOR_FLUX,   PARAM_CUR_KP,
    PARAM_CUR_KI,        PARAM_SPD_KP,       PARAM_SPD_KI,
    PARAM_POS_KP,        PARAM_LIMIT_TORQUE, PARAM_LIMIT_CURRENT,
    PARAM_LIMIT_SPEED,   PARAM_VEL_MAX,      PARAM_ACC_SET,
    PARAM_ACC_RAD,       PARAM_INERTIA,      PARAM_ADD_OFFSET,
    PARAM_OV_THRESHOLD,  PARAM_UV_THRESHOLD, PARAM_OC_THRESHOLD,
    PARAM_OT_THRESHOLD,  PARAM_SMO_ALPHA,    PARAM_SMO_BETA,
    PARAM_FF_FRICTION,   PARAM_FW_MAX_CUR,   PARAM_FW_START_VEL,
    PARAM_COGGING_EN,    PARAM_LADRC_ENABLE, PARAM_LADRC_OMEGA_O,
    PARAM_LADRC_OMEGA_C, PARAM_LADRC_B0,     PARAM_LADRC_MAX_OUT,
};

static const ParamIndex s_restore_uint8_indices[] = {
    PARAM_MOTOR_POLE_PAIRS, PARAM_CAN_BAUDRATE, PARAM_PROTOCOL_TYPE,
    PARAM_ZERO_STA,         PARAM_DAMPER,       PARAM_RUN_MODE,
};

static ParamEntry s_restore_float_entry = {
    .type = PARAM_TYPE_FLOAT,
    .access = PARAM_ACCESS_RW,
    .ptr = &s_float_value_2,
    .min = -1000.0f,
    .max = 1000.0f,
};

static ParamEntry s_restore_uint8_entry = {
    .type = PARAM_TYPE_UINT8,
    .access = PARAM_ACCESS_RW,
    .ptr = &s_uint8_value_2,
    .min = 0.0f,
    .max = 127.0f,
};

static bool IndexInList(uint16_t index, const ParamIndex *indices,
                        size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (indices[i] == index) {
      return true;
    }
  }
  return false;
}

const ParamEntry *ParamTable_Find(uint16_t index) {
  for (size_t i = 0; i < sizeof(s_entries) / sizeof(s_entries[0]); ++i) {
    if (s_entries[i].index == index) {
      return &s_entries[i];
    }
  }
  if (IndexInList(index, s_restore_float_indices,
                  sizeof(s_restore_float_indices) /
                      sizeof(s_restore_float_indices[0]))) {
    s_restore_float_entry.index = index;
    return &s_restore_float_entry;
  }
  if (IndexInList(index, s_restore_uint8_indices,
                  sizeof(s_restore_uint8_indices) /
                      sizeof(s_restore_uint8_indices[0]))) {
    s_restore_uint8_entry.index = index;
    s_restore_uint8_entry.max = (index == PARAM_RUN_MODE) ? 5.0f : 127.0f;
    return &s_restore_uint8_entry;
  }
  return NULL;
}

void ParamTable_Init(void) {}
uint32_t ParamTable_GetCount(void) {
  return (uint32_t)(sizeof(s_entries) / sizeof(s_entries[0]));
}
const ParamEntry *ParamTable_GetTable(void) { return s_entries; }

void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)code;
  (void)message;
  (void)file;
  (void)line;
  s_error_count++;
}

FlashStorageResult ParamStorage_Save(FlashParamData *data) {
  if (data != NULL) {
    s_storage_image = *data;
  }
  s_storage_save_count++;
  return s_storage_result;
}

FlashStorageResult ParamStorage_Load(FlashParamData *data) {
  *data = s_storage_image;
  return FLASH_STORAGE_OK;
}

void ParamStorage_Init(void) {}
bool ParamStorage_HasValidData(void) { return false; }

static void ResetState(void) {
  s_float_value = 2.5f;
  s_float_value_2 = 1.25f;
  s_uint8_value = 7U;
  s_uint8_value_2 = 1U;
  s_uint16_value = 100U;
  s_uint32_value = 1000U;
  s_int32_value = -1000;
  s_storage_result = FLASH_STORAGE_OK;
  memset(&s_storage_image, 0, sizeof(s_storage_image));
  s_storage_image.motor_rs = 2.5f;
  s_storage_image.motor_ls = 1.0f;
  s_storage_image.motor_flux = 1.0f;
  s_storage_image.motor_pole_pairs = 7U;
  s_storage_image.cur_kp = 1.0f;
  s_storage_image.cur_ki = 1.0f;
  s_storage_image.spd_kp = 1.0f;
  s_storage_image.spd_ki = 1.0f;
  s_storage_image.pos_kp = 1.0f;
  s_storage_image.limit_torque = 1.0f;
  s_storage_image.limit_current = 1.0f;
  s_storage_image.limit_speed = 1.0f;
  s_storage_image.vel_max = 100.0f;
  s_storage_image.acc_set = 100.0f;
  s_storage_image.acc_rad = 1.0f;
  s_storage_image.inertia = 1.0f;
  s_storage_image.can_id = 7U;
  s_storage_image.can_baudrate = 1U;
  s_storage_image.protocol_type = 1U;
  s_storage_image.zero_sta = 1U;
  s_storage_image.add_offset = 1.0f;
  s_storage_image.damper = 1U;
  s_storage_image.can_timeout = 1000U;
  s_storage_image.run_mode = 1U;
  s_storage_image.over_voltage_threshold = 1.0f;
  s_storage_image.under_voltage_threshold = 1.0f;
  s_storage_image.over_current_threshold = 1.0f;
  s_storage_image.over_temp_threshold = 1.0f;
  s_storage_image.smo_alpha = 1.0f;
  s_storage_image.smo_beta = 1.0f;
  s_storage_image.ff_friction = 1.0f;
  s_storage_image.fw_max_current = 1.0f;
  s_storage_image.fw_start_velocity = 1.0f;
  s_storage_image.cogging_comp_enabled = 1.0f;
  s_storage_image.ladrc_enable = 0.0f;
  s_storage_image.ladrc_omega_o = 1000.0f;
  s_storage_image.ladrc_omega_c = 300.0f;
  s_storage_image.ladrc_b0 = 1.0f;
  s_storage_image.ladrc_max_output = 10.0f;
}

static int TestReadMismatchDoesNotOverwrite(void) {
  struct {
    uint8_t before;
    uint8_t value;
    uint8_t after[6];
  } guarded = {0xA5U, 0x5AU, {0xC3U, 0xC3U, 0xC3U, 0xC3U, 0xC3U, 0xC3U}};
  const unsigned char expected[] = {0xA5U, 0x5AU, 0xC3U, 0xC3U,
                                    0xC3U, 0xC3U, 0xC3U, 0xC3U};

  if (Param_ReadUint8(PARAM_MOTOR_RS, &guarded.value) !=
      PARAM_ERR_INVALID_TYPE) {
    return 1;
  }
  return memcmp(&guarded, expected, sizeof(expected)) != 0;
}

static int TestWriteMismatchDoesNotModifyTarget(void) {
  float original_float = s_float_value;
  uint8_t original_uint8 = s_uint8_value;

  if (Param_WriteUint8(PARAM_MOTOR_RS, 99U) != PARAM_ERR_INVALID_TYPE ||
      s_float_value != original_float) {
    return 1;
  }
  if (Param_WriteFloat(PARAM_CAN_ID, 42.0f) != PARAM_ERR_INVALID_TYPE ||
      s_uint8_value != original_uint8) {
    return 1;
  }
  return 0;
}

static int TestCorrectTypedAccess(void) {
  uint32_t timeout = 0U;
  if (Param_WriteUint32(PARAM_CAN_TIMEOUT, 2500U) != PARAM_OK ||
      Param_ReadUint32(PARAM_CAN_TIMEOUT, &timeout) != PARAM_OK ||
      timeout != 2500U) {
    return 1;
  }
  return 0;
}

static int TestWriteFromFloatRejectsNonFiniteValues(void) {
  ResetState();
  if (Param_WriteFromFloat(PARAM_MOTOR_RS, NAN) != PARAM_ERR_OUT_OF_RANGE ||
      Param_WriteFromFloat(PARAM_MOTOR_RS, INFINITY) !=
          PARAM_ERR_OUT_OF_RANGE ||
      s_float_value != 2.5f) {
    return 1;
  }
  return 0;
}

static int TestWriteFromFloatRejectsFractionalIntegerValues(void) {
  ResetState();
  if (Param_WriteFromFloat(PARAM_CAN_ID, 7.5f) != PARAM_ERR_OUT_OF_RANGE ||
      Param_WriteFromFloat(TEST_PARAM_UINT16, 99.25f) !=
          PARAM_ERR_OUT_OF_RANGE ||
      Param_WriteFromFloat(PARAM_CAN_TIMEOUT, 1000.5f) !=
          PARAM_ERR_OUT_OF_RANGE ||
      Param_WriteFromFloat(TEST_PARAM_INT32, -12.5f) !=
          PARAM_ERR_OUT_OF_RANGE) {
    return 1;
  }
  return s_uint8_value != 7U || s_uint16_value != 100U ||
         s_uint32_value != 1000U || s_int32_value != -1000;
}

static int TestWriteFromFloatAcceptsIntegralValues(void) {
  ResetState();
  if (Param_WriteFromFloat(PARAM_CAN_ID, 42.0f) != PARAM_OK ||
      Param_WriteFromFloat(TEST_PARAM_UINT16, 321.0f) != PARAM_OK ||
      Param_WriteFromFloat(PARAM_CAN_TIMEOUT, 4096.0f) != PARAM_OK ||
      Param_WriteFromFloat(TEST_PARAM_INT32, -321.0f) != PARAM_OK) {
    return 1;
  }
  return s_uint8_value != 42U || s_uint16_value != 321U ||
         s_uint32_value != 4096U || s_int32_value != -321;
}

static int TestReadAsFloatRejectsUnrepresentableIntegerValues(void) {
  float value = 0.0f;
  s_uint32_value = 16777217U;
  if (Param_ReadAsFloat(PARAM_CAN_TIMEOUT, &value) != PARAM_ERR_OUT_OF_RANGE) {
    return 1;
  }
  s_int32_value = 16777217;
  if (Param_ReadAsFloat(TEST_PARAM_INT32, &value) != PARAM_ERR_OUT_OF_RANGE) {
    return 1;
  }
  s_uint32_value = 4096U;
  s_int32_value = -4096;
  if (Param_ReadAsFloat(PARAM_CAN_TIMEOUT, &value) != PARAM_OK ||
      value != 4096.0f) {
    return 1;
  }
  if (Param_ReadAsFloat(TEST_PARAM_INT32, &value) != PARAM_OK ||
      value != -4096.0f) {
    return 1;
  }
  return 0;
}

static int TestLoadRejectsOutOfRangeImageWithoutPartialApply(void) {
  ResetState();
  s_storage_image.motor_rs = 99.0f;
  if (Param_LoadFromFlash() != PARAM_ERR_OUT_OF_RANGE) {
    return 1;
  }
  return s_float_value != 2.5f || s_uint8_value != 7U ||
         s_uint32_value != 1000U;
}

static int TestLoadRejectsNaNImageWithoutPartialApply(void) {
  ResetState();
  s_storage_image.add_offset = NAN;
  if (Param_LoadFromFlash() != PARAM_ERR_OUT_OF_RANGE) {
    return 1;
  }
  return s_float_value != 2.5f || s_uint8_value != 7U ||
         s_uint32_value != 1000U;
}

static int TestScheduledSaveRetriesFailure(void) {
  int initial_count = s_storage_save_count;
  s_storage_result = FLASH_STORAGE_OK;
  Param_ScheduleSave();
  if (!Param_ProcessScheduledSave() ||
      s_storage_save_count != initial_count + 1) {
    return 1;
  }
  if (Param_ProcessScheduledSave() ||
      s_storage_save_count != initial_count + 1) {
    return 1;
  }

  s_storage_result = FLASH_STORAGE_ERR_WRITE;
  Param_ScheduleSave();
  if (Param_ProcessScheduledSave() ||
      s_storage_save_count != initial_count + 2) {
    return 1;
  }
  s_storage_result = FLASH_STORAGE_OK;
  if (!Param_ProcessScheduledSave() ||
      s_storage_save_count != initial_count + 3) {
    return 1;
  }
  return 0;
}

int main(void) {
  ResetState();
  if (TestReadMismatchDoesNotOverwrite()) {
    printf("FAIL typed read overwrote a smaller destination\n");
    return 1;
  }
  if (TestWriteMismatchDoesNotModifyTarget()) {
    printf("FAIL typed write modified a mismatched target\n");
    return 1;
  }
  if (TestCorrectTypedAccess()) {
    printf("FAIL correct typed access\n");
    return 1;
  }
  if (TestWriteFromFloatRejectsNonFiniteValues()) {
    printf("FAIL wire writes accepted non-finite floats\n");
    return 1;
  }
  if (TestWriteFromFloatRejectsFractionalIntegerValues()) {
    printf("FAIL wire writes accepted fractional integer values\n");
    return 1;
  }
  if (TestWriteFromFloatAcceptsIntegralValues()) {
    printf("FAIL wire writes rejected valid integral values\n");
    return 1;
  }
  if (TestReadAsFloatRejectsUnrepresentableIntegerValues()) {
    printf("FAIL wire reads accepted imprecise integer values\n");
    return 1;
  }
  if (TestLoadRejectsOutOfRangeImageWithoutPartialApply()) {
    printf("FAIL invalid flash range was accepted or partially applied\n");
    return 1;
  }
  if (TestLoadRejectsNaNImageWithoutPartialApply()) {
    printf("FAIL invalid flash NaN was accepted or partially applied\n");
    return 1;
  }
  if (TestScheduledSaveRetriesFailure()) {
    printf("FAIL scheduled save request was lost or duplicated\n");
    return 1;
  }
  if (s_error_count < 9) {
    printf("FAIL type and validation errors were not reported\n");
    return 1;
  }
  printf("Parameter typed access safety tests passed\n");
  return 0;
}
