#ifndef TEST_MOCK_PARAM_ACCESS_H
#define TEST_MOCK_PARAM_ACCESS_H

#include "param_table.h"

typedef enum {
  PARAM_OK = 0,
  PARAM_ERR_INVALID_INDEX,
  PARAM_ERR_INVALID_TYPE,
  PARAM_ERR_READONLY,
  PARAM_ERR_OUT_OF_RANGE,
  PARAM_ERR_NULL_PTR,
} ParamResult;

ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry);
ParamResult Param_Write(uint16_t index, const void *data);
ParamResult Param_WriteFloat(uint16_t index, float value);
ParamResult Param_ReadFloat(uint16_t index, float *value);
ParamResult Param_WriteUint8(uint16_t index, uint8_t value);
void Param_ScheduleSave(void);

#endif
