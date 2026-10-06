// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/* Parameter/Flash compatibility at the BSP boundary. Logical write slots record
 * the requested addresses and bytes; this is not an STM32 page-geometry model. */
#include "parameter_access.h"
#include "parameter_storage.h"
#include "motor_runtime.h"
#include "fault_monitor.h"
#include "board_flash.h"
#include "error_manager.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CHECK(expr) do { \
  if (!(expr)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    return 1; \
  } \
} while (0)

MOTOR_DATA motor_data;
DetectionConfig s_config;
uint8_t g_can_id, g_can_baudrate, g_protocol_type;
uint32_t g_can_timeout_ms;
uint8_t g_zero_sta, g_damper_enable, g_run_mode;
float g_add_offset;

#include "param_support/legacy_params.h"

static uint8_t slots[2][FLASH_PARAM_PAGE_SIZE];
static unsigned active_slot;
static unsigned erase_calls, write_calls, verify_calls, unlock_calls, lock_calls;
static unsigned fail_erase_call, fail_write_call, fail_verify_call;
static uint32_t crc_length, last_error;
static const uint32_t addresses[] = {FLASH_PARAM_PAGE1_ADDR, FLASH_PARAM_PAGE2_ADDR};

static unsigned slot_at(uint32_t address) {
  assert(address == addresses[0] || address == addresses[1]);
  return address == addresses[1];
}
static void reset_flash(void) {
  memset(slots, 0xFF, sizeof(slots));
  active_slot = 0;
  erase_calls = write_calls = verify_calls = unlock_calls = lock_calls = 0;
  fail_erase_call = fail_write_call = fail_verify_call = 0;
  last_error = crc_length = 0;
}
void BSP_Flash_Unlock(void) { ++unlock_calls; }
void BSP_Flash_Lock(void) { ++lock_calls; }
bool BSP_Flash_ErasePage(uint32_t address) {
  active_slot = slot_at(address);
  ++erase_calls;
  if (erase_calls == fail_erase_call) return false;
  memset(slots[active_slot], 0xFF, sizeof(slots[active_slot]));
  return true;
}
bool BSP_Flash_WriteDoubleWord(uint32_t address, uint64_t data) {
  unsigned offset = address - addresses[active_slot];
  assert(offset + 8 <= FLASH_PARAM_PAGE_SIZE);
  ++write_calls;
  if (write_calls == fail_write_call) return false;
  memcpy(slots[active_slot] + offset, &data, 8);
  return true;
}
void BSP_Flash_Read(uint32_t address, uint8_t *data, uint32_t length) {
  assert(length <= FLASH_PARAM_PAGE_SIZE);
  memcpy(data, slots[slot_at(address)], length);
}
bool BSP_Flash_Verify(uint32_t address, const uint8_t *data, uint32_t length) {
  ++verify_calls;
  return verify_calls != fail_verify_call &&
         memcmp(slots[slot_at(address)], data, length) == 0;
}
uint32_t BSP_Flash_CalculateCRC32(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  crc_length = length;
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
  }
  return crc ^ 0xFFFFFFFFu;
}
void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)message; (void)file; (void)line; last_error = code;
}

static size_t value_size(ParamType type) {
  return type == PARAM_TYPE_UINT8 ? 1 : type == PARAM_TYPE_UINT16 ? 2 : 4;
}
static void seed_params(float fraction) {
  const ParamEntry *table = ParamTable_GetTable();
  for (unsigned i = 0; i < ParamTable_GetCount(); ++i) {
    const ParamEntry *entry = &table[i];
    float value = entry->min + (entry->max - entry->min) * fraction;
    if (entry->type == PARAM_TYPE_FLOAT) *(float *)entry->ptr = value;
    else if (entry->type == PARAM_TYPE_UINT8) *(uint8_t *)entry->ptr = (uint8_t)value;
    else if (entry->type == PARAM_TYPE_UINT32) *(uint32_t *)entry->ptr = 20001u;
  }
}
static void snapshot(uint8_t values[40][4]) {
  memset(values, 0, 40 * 4);
  const ParamEntry *table = ParamTable_GetTable();
  for (unsigned i = 0; i < 40; ++i)
    memcpy(values[i], table[i].ptr, value_size(table[i].type));
}
static void finish_image(FlashParamData *image) {
  image->magic = FLASH_MAGIC_WORD;
  image->version = FLASH_PARAM_VERSION;
  image->crc32 = 0;
  image->crc32 = BSP_Flash_CalculateCRC32((uint8_t *)image + 16, sizeof(*image) - 16);
}

static int test_schema_and_layout(void) {
  CHECK(check_legacy_layout() == 0);
  CHECK(ParamTable_GetCount() == 40);
  CHECK(sizeof(legacy_schema) / sizeof(legacy_schema[0]) == 40);
  const ParamEntry *table = ParamTable_GetTable();
  unsigned persistent = 0;
  for (unsigned i = 0; i < 40; ++i) {
    const ParamEntry *entry = &table[i], *expected = &legacy_schema[i];
    CHECK(entry->index == expected->index && entry->type == expected->type);
    CHECK(entry->ptr == expected->ptr && strcmp(entry->name, expected->name) == 0);
    CHECK(entry->attr == expected->attr && entry->access == expected->access);
    CHECK(entry->min == expected->min && entry->max == expected->max);
    CHECK(entry->default_val == expected->default_val && entry->need_save == expected->need_save);
    CHECK(ParamTable_Find(entry->index) == entry);
    persistent += (entry->attr & PARAM_ATTR_PERSISTENT) != 0;
  }
  CHECK(persistent == 39);
  CHECK(FLASH_PARAM_PAGE1_ADDR == 0x0801F800 && FLASH_PARAM_PAGE2_ADDR == 0x0801FC00);
  CHECK(FLASH_MAGIC_WORD == 0x464F4331 && FLASH_PARAM_VERSION == 0x00010000);
  CHECK(BSP_Flash_CalculateCRC32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
  return 0;
}

static int test_save_restore_matches_legacy(void) {
  /* Exercise each field at three different values, including both range ends. */
  const float fractions[] = {0.0f, 0.375f, 1.0f};
  for (unsigned sample = 0; sample < 3; ++sample) {
    reset_flash();
    seed_params(fractions[sample]);
    uint8_t originally_saved[40][4], before_restore[40][4];
    uint8_t legacy_restored[40][4], actual_restored[40][4];
    snapshot(originally_saved);
    FlashParamData expected;
    Legacy_Collect(&expected);
    finish_image(&expected);
    CHECK(Param_SaveToFlash() == PARAM_OK);
    CHECK(memcmp(slots[0], &expected, sizeof(expected)) == 0);
    CHECK(memcmp(slots[1], &expected, sizeof(expected)) == 0);
    CHECK(crc_length == sizeof(expected) - 16);
    CHECK(erase_calls == 2 && write_calls == 2 * ((sizeof(expected) + 7) / 8));
    CHECK(verify_calls == 1 && unlock_calls == 1 && lock_calls == 1);
    CHECK(expected.cur_filt_gain == 0 && expected.spd_filt_gain == 0);
    CHECK(expected.can_timeout == 20001u); /* Above the 10000 ms API limit. */
    seed_params(0.625f);
    snapshot(before_restore);
    Legacy_Restore(&expected);
    snapshot(legacy_restored);
    seed_params(0.625f);
    CHECK(Param_LoadFromFlash() == PARAM_OK);
    snapshot(actual_restored);
    CHECK(memcmp(legacy_restored, actual_restored, sizeof(actual_restored)) == 0);
    unsigned saved_count = 0;
    for (unsigned i = 0; i < 40; ++i) {
      uint16_t id = legacy_schema[i].index;
      if (id == PARAM_COGGING_CALIB || (id >= PARAM_LADRC_ENABLE && id <= PARAM_LADRC_MAX_OUT)) {
        CHECK(memcmp(actual_restored[i], before_restore[i], 4) == 0);
      } else {
        ++saved_count;
        CHECK(memcmp(actual_restored[i], originally_saved[i], 4) == 0);
      }
    }
    CHECK(saved_count == 34);
    CHECK(g_can_timeout_ms == 20001u);
  }
  return 0;
}

static int test_restore_and_access_exceptions(void) {
  seed_params(0.5f);
  FlashParamData image;
  Legacy_Collect(&image);
  image.motor_rs = 99.0f; /* Invalid float is skipped, unlike CAN timeout. */
  image.can_timeout = 50001u;
  finish_image(&image);
  reset_flash();
  memcpy(slots[0], &image, sizeof(image));
  uint8_t expected[40][4], actual[40][4];
  seed_params(0.125f); Legacy_Restore(&image); snapshot(expected);
  seed_params(0.125f); CHECK(Param_LoadFromFlash() == PARAM_OK); snapshot(actual);
  CHECK(memcmp(expected, actual, sizeof(actual)) == 0);
  CHECK(g_can_timeout_ms == 50001u);
  uint32_t invalid_timeout = 50001u;
  CHECK(Param_Write(PARAM_CAN_TIMEOUT, &invalid_timeout) == PARAM_ERR_OUT_OF_RANGE);

  g_can_id = 42;
  union { float value; uint8_t bytes[4]; } readback;
  memset(&readback, 0xA5, sizeof(readback));
  CHECK(Param_ReadFloat(PARAM_CAN_ID, &readback.value) == PARAM_ERR_INVALID_TYPE);
  CHECK(readback.bytes[0] == 42 && readback.bytes[1] == 0xA5);
  CHECK(readback.bytes[2] == 0xA5 && readback.bytes[3] == 0xA5);
  CHECK(Param_WriteFloat(PARAM_CAN_ID, 42.0f) == PARAM_ERR_OUT_OF_RANGE);
  CHECK(g_can_id == 42); /* Float API still does not convert uint8 parameters. */
  return 0;
}

static int test_storage_failures_and_deferred_save(void) {
  seed_params(0.5f);
  FlashParamData image, loaded;
  Legacy_Collect(&image); finish_image(&image);
  reset_flash(); fail_erase_call = 1;
  CHECK(ParamStorage_Save(&image) == FLASH_STORAGE_ERR_ERASE);
  CHECK(write_calls == 0 && lock_calls == 1);
  reset_flash(); fail_write_call = 1;
  CHECK(ParamStorage_Save(&image) == FLASH_STORAGE_ERR_WRITE && lock_calls == 1);
  reset_flash(); fail_verify_call = 1;
  CHECK(ParamStorage_Save(&image) == FLASH_STORAGE_ERR_VERIFY);
  CHECK(erase_calls == 1 && lock_calls == 1);
  reset_flash(); fail_erase_call = 2;
  CHECK(ParamStorage_Save(&image) == FLASH_STORAGE_ERR_ERASE && lock_calls == 1);
  reset_flash();
  memcpy(slots[1], &image, sizeof(image));
  CHECK(ParamStorage_Load(&loaded) == FLASH_STORAGE_OK); /* Missing page-one magic. */
  memcpy(slots[0], &image, sizeof(image));
  slots[0][offsetof(FlashParamData, motor_rs)] ^= 1;
  CHECK(ParamStorage_HasValidData()); /* Existing check only inspects magic. */
  CHECK(ParamStorage_Load(&loaded) == FLASH_STORAGE_ERR_CRC); /* No CRC fallback. */
  reset_flash();
  image.version = 0x76543210; /* The current loader accepts a different version. */
  memcpy(slots[0], &image, sizeof(image));
  CHECK(ParamStorage_Load(&loaded) == FLASH_STORAGE_OK);
  CHECK(ParamStorage_Save(NULL) == FLASH_STORAGE_ERR_LOCKED);
  CHECK(ParamStorage_Load(NULL) == FLASH_STORAGE_ERR_LOCKED);
  reset_flash();
  CHECK(!ParamStorage_HasValidData());
  CHECK(Param_LoadFromFlash() == PARAM_ERR_INVALID_INDEX);
  Param_ScheduleSave();
  CHECK(write_calls == 0);
  fail_erase_call = 1;
  CHECK(Param_ProcessScheduledSave()); /* A consumed request remains true on failure. */
  CHECK(!Param_ProcessScheduledSave() && last_error != 0);
  return 0;
}

int main(void) {
  int failed = 0;
  failed += test_schema_and_layout();
  failed += test_save_restore_matches_legacy();
  failed += test_restore_and_access_exceptions();
  failed += test_storage_failures_and_deferred_save();
  if (failed) return 1;
  puts("Parameter compatibility: 4 groups passed (40 entries, 39 persistent, 34 saved).");
  return 0;
}
