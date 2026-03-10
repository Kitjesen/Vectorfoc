#ifndef TEST_MOCK_PARAM_TABLE_H
#define TEST_MOCK_PARAM_TABLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  PARAM_CAN_ID = 0x3000,
  PARAM_CAN_BAUDRATE = 0x3001,
  PARAM_PROTOCOL_TYPE = 0x3002,
  PARAM_CAN_TIMEOUT = 0x3003,
  PARAM_RUN_MODE = 0x3030,
} ParamIndex;

typedef enum {
  PARAM_TYPE_FLOAT = 0,
  PARAM_TYPE_UINT8,
  PARAM_TYPE_UINT16,
  PARAM_TYPE_UINT32,
  PARAM_TYPE_INT32,
} ParamType;

#define PARAM_ACCESS_R 0x01
#define PARAM_ACCESS_W 0x02
#define PARAM_ACCESS_RW (PARAM_ACCESS_R | PARAM_ACCESS_W)

typedef struct {
  uint16_t index;
  ParamType type;
  uint8_t attr;
  uint8_t access;
  const char *name;
  void *ptr;
  float min;
  float max;
  float default_val;
  bool need_save;
} ParamEntry;

const ParamEntry *ParamTable_Find(uint16_t index);
uint32_t ParamTable_GetCount(void);
const ParamEntry *ParamTable_GetTable(void);

#endif
