#include "parameter_access.h"
#include "encoder_calibration_storage_internal.h"
#include "parameter_storage.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

#define TEST_UINT16 0x7ffeu
#define TEST_INT32 0x7fffu

static float float_value = 2.5f;
static uint8_t uint8_value = 7u;
static uint16_t uint16_value = 100u;
static uint32_t uint32_value = 1000u;
static int32_t int32_value = -1000;

static const ParamEntry entries[] = {
    {.index = PARAM_MOTOR_RS, .type = PARAM_TYPE_FLOAT, .access = PARAM_ACCESS_RW,
     .ptr = &float_value, .min = 0.0f, .max = 10.0f, .default_val = 2.5f},
    {.index = PARAM_CAN_ID, .type = PARAM_TYPE_UINT8, .access = PARAM_ACCESS_RW,
     .ptr = &uint8_value, .min = 1.0f, .max = 127.0f, .default_val = 7.0f},
    {.index = TEST_UINT16, .type = PARAM_TYPE_UINT16, .access = PARAM_ACCESS_RW,
     .ptr = &uint16_value, .min = 0.0f, .max = 1000.0f, .default_val = 100.0f},
    {.index = PARAM_CAN_TIMEOUT, .type = PARAM_TYPE_UINT32,
     .access = PARAM_ACCESS_RW, .ptr = &uint32_value, .min = 0.0f,
     .max = 10000.0f, .default_val = 1000.0f},
    {.index = TEST_INT32, .type = PARAM_TYPE_INT32, .access = PARAM_ACCESS_RW,
     .ptr = &int32_value, .min = -2000.0f, .max = 2000.0f,
     .default_val = -1000.0f},
};

const ParamEntry *ParamTable_Find(uint16_t index) {
  for (unsigned i = 0; i < sizeof(entries) / sizeof(entries[0]); ++i)
    if (entries[i].index == index)
      return &entries[i];
  return NULL;
}
bool ParamTable_IsBound(void) { return true; }
void ParamTable_Init(void) {}
uint32_t ParamTable_GetCount(void) {
  return sizeof(entries) / sizeof(entries[0]);
}
const ParamEntry *ParamTable_GetTable(void) { return entries; }
ParamResult ParamTable_GetBinding(const ParamEntry *entry,
                                  ParamTargetBinding *binding) {
  if (entry == NULL || binding == NULL || entry->ptr == NULL)
    return PARAM_ERR_NULL_PTR;
  *binding = (ParamTargetBinding){.index = entry->index,
                                  .type = entry->type,
                                  .target = entry->ptr,
                                  .default_val = entry->default_val};
  return PARAM_OK;
}

void ParamStorage_Init(void) {}
FlashStorageResult ParamStorage_Save(FlashParamData *data) {
  (void)data;
  return FLASH_STORAGE_OK;
}
FlashStorageResult ParamStorage_Load(FlashParamData *data) {
  (void)data;
  return FLASH_STORAGE_ERR_MAGIC;
}
bool ParamStorage_HasValidData(void) { return false; }

void ParamEncoderCalibration_Collect(FlashParamData *data) { (void)data; }
void ParamEncoderCalibration_Restore(const FlashParamData *data) { (void)data; }
void ParamEncoderCalibration_Clear(void) {}
bool ParamEncoderCalibration_IsFlashDataValid(const FlashParamData *data) {
  (void)data;
  return true;
}

void ErrorManager_ReportFull(uint32_t code, const char *message,
                             const char *file, uint32_t line) {
  (void)code;
  (void)message;
  (void)file;
  (void)line;
}

static void wrong_type_never_writes_target_or_destination(void) {
  uint8_t guarded[8] = {0xa5u, 0x5au, 0xc3u, 0xc3u,
                        0xc3u, 0xc3u, 0xc3u, 0xc3u};
  const float original_float = float_value;

  assert(Param_ReadUint8(PARAM_MOTOR_RS, &guarded[1]) ==
         PARAM_ERR_INVALID_TYPE);
  assert(guarded[0] == 0xa5u && guarded[1] == 0x5au && guarded[7] == 0xc3u);
  assert(Param_WriteUint8(PARAM_MOTOR_RS, 99u) == PARAM_ERR_INVALID_TYPE);
  assert(float_value == original_float);
}

static void float_wire_conversion_rejects_invalid_integer_values(void) {
  assert(Param_WriteFromFloat(PARAM_CAN_ID, 7.5f) == PARAM_ERR_OUT_OF_RANGE);
  assert(Param_WriteFromFloat(TEST_UINT16, 99.25f) == PARAM_ERR_OUT_OF_RANGE);
  assert(Param_WriteFromFloat(PARAM_CAN_TIMEOUT, INFINITY) ==
         PARAM_ERR_OUT_OF_RANGE);
  assert(Param_WriteFromFloat(TEST_INT32, NAN) == PARAM_ERR_OUT_OF_RANGE);
  assert(uint8_value == 7u && uint16_value == 100u &&
         uint32_value == 1000u && int32_value == -1000);
}

static void float_wire_conversion_accepts_exact_integral_values(void) {
  assert(Param_WriteFromFloat(PARAM_CAN_ID, 42.0f) == PARAM_OK);
  assert(Param_WriteFromFloat(TEST_UINT16, 321.0f) == PARAM_OK);
  assert(Param_WriteFromFloat(PARAM_CAN_TIMEOUT, 4096.0f) == PARAM_OK);
  assert(Param_WriteFromFloat(TEST_INT32, -321.0f) == PARAM_OK);
  assert(uint8_value == 42u && uint16_value == 321u &&
         uint32_value == 4096u && int32_value == -321);
}

static void read_as_float_rejects_lossy_integer(void) {
  float wire = 0.0f;
  uint32_value = 16777217u;
  assert(Param_ReadAsFloat(PARAM_CAN_TIMEOUT, &wire) == PARAM_ERR_OUT_OF_RANGE);
  uint32_value = 4096u;
  assert(Param_ReadAsFloat(PARAM_CAN_TIMEOUT, &wire) == PARAM_OK);
  assert(wire == 4096.0f);
}

int main(void) {
  wrong_type_never_writes_target_or_destination();
  float_wire_conversion_rejects_invalid_integer_values();
  float_wire_conversion_accepts_exact_integral_values();
  read_as_float_rejects_lossy_integer();
  return 0;
}
