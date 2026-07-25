// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "param_table.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PARAM_TABLE_TEST_MAX_ENTRIES 48U

typedef union {
  float float_value;
  uint8_t uint8_value;
  uint16_t uint16_value;
  uint32_t uint32_value;
  int32_t int32_value;
} ParamTableTestValue;

static ParamTableTestValue s_values[PARAM_TABLE_TEST_MAX_ENTRIES];
static ParamTargetBinding s_bindings[PARAM_TABLE_TEST_MAX_ENTRIES];
static uint32_t s_binding_count;

typedef enum {
  PARAM_TABLE_TEST_BINDING_VALID,
  PARAM_TABLE_TEST_BINDING_OMIT_LAST,
  PARAM_TABLE_TEST_BINDING_INDEX_MISMATCH,
  PARAM_TABLE_TEST_BINDING_TYPE_MISMATCH,
  PARAM_TABLE_TEST_BINDING_NULL_TARGET,
  PARAM_TABLE_TEST_BINDING_NAN_DEFAULT,
} ParamTableTestBindingMode;

typedef struct {
  ParamTableTestBindingMode mode;
} ParamTableTestResolver;

static ParamTableTestResolver s_valid_resolver = {
    .mode = PARAM_TABLE_TEST_BINDING_VALID};

static float ParamTableTest_DefaultFor(const ParamEntry *entry) {
  if (entry->type == PARAM_TYPE_FLOAT) {
    return entry->min + ((entry->max - entry->min) * 0.5f);
  }
  return entry->min;
}

static int ParamTableTest_BuildBindings(void) {
  const ParamEntry *table = ParamTable_GetTable();
  s_binding_count = ParamTable_GetCount();
  if (table == NULL || s_binding_count == 0U ||
      s_binding_count > PARAM_TABLE_TEST_MAX_ENTRIES) {
    return 1;
  }

  memset(s_values, 0, sizeof(s_values));
  for (uint32_t i = 0U; i < s_binding_count; ++i) {
    s_bindings[i] = (ParamTargetBinding){
        .index = table[i].index,
        .type = table[i].type,
        .target = &s_values[i],
        .default_val = ParamTableTest_DefaultFor(&table[i]),
    };
  }
  return 0;
}

static bool ParamTableTest_Resolve(void *context, uint16_t index,
                                   ParamType type,
                                   ParamTargetBinding *binding) {
  ParamTableTestResolver *resolver = context;
  if (resolver == NULL || binding == NULL) {
    return false;
  }

  uint32_t available_count = s_binding_count;
  if (resolver->mode == PARAM_TABLE_TEST_BINDING_OMIT_LAST) {
    available_count--;
  }
  for (uint32_t i = 0U; i < available_count; ++i) {
    if (s_bindings[i].index != index) {
      continue;
    }

    *binding = s_bindings[i];
    switch (resolver->mode) {
    case PARAM_TABLE_TEST_BINDING_INDEX_MISMATCH:
      binding->index = 0U;
      break;
    case PARAM_TABLE_TEST_BINDING_TYPE_MISMATCH:
      binding->type =
          type == PARAM_TYPE_FLOAT ? PARAM_TYPE_UINT8 : PARAM_TYPE_FLOAT;
      break;
    case PARAM_TABLE_TEST_BINDING_NULL_TARGET:
      binding->target = NULL;
      break;
    case PARAM_TABLE_TEST_BINDING_NAN_DEFAULT:
      binding->default_val = NAN;
      break;
    default:
      break;
    }
    return true;
  }
  return false;
}

static int ParamTableTest_AssertValuesInitialized(void) {
  const ParamEntry *table = ParamTable_GetTable();
  for (uint32_t i = 0U; i < s_binding_count; ++i) {
    const ParamEntry *entry = &table[i];
    ParamTargetBinding binding = {0};
    if (ParamTable_Find(entry->index) != entry ||
        ParamTable_GetBinding(entry, &binding) != PARAM_OK ||
        binding.target != s_bindings[i].target ||
        binding.default_val != s_bindings[i].default_val) {
      return 1;
    }

    switch (entry->type) {
    case PARAM_TYPE_FLOAT:
      if (s_values[i].float_value != s_bindings[i].default_val)
        return 1;
      break;
    case PARAM_TYPE_UINT8:
      if (s_values[i].uint8_value != (uint8_t)s_bindings[i].default_val)
        return 1;
      break;
    case PARAM_TYPE_UINT16:
      if (s_values[i].uint16_value != (uint16_t)s_bindings[i].default_val)
        return 1;
      break;
    case PARAM_TYPE_UINT32:
      if (s_values[i].uint32_value != (uint32_t)s_bindings[i].default_val)
        return 1;
      break;
    case PARAM_TYPE_INT32:
      if (s_values[i].int32_value != (int32_t)s_bindings[i].default_val)
        return 1;
      break;
    }
  }
  return 0;
}

static int TestParamTableRequiresCompleteBindings(void) {
  if (ParamTable_IsBound() || ParamTableTest_BuildBindings()) {
    return 1;
  }

  ParamTableTestResolver incomplete_resolver = {
      .mode = PARAM_TABLE_TEST_BINDING_OMIT_LAST};
  if (ParamTable_SetBindingAdapter(ParamTableTest_Resolve,
                                   &incomplete_resolver) !=
          PARAM_ERR_INVALID_INDEX ||
      ParamTable_IsBound()) {
    return 1;
  }

  if (ParamTable_SetBindingAdapter(ParamTableTest_Resolve, &s_valid_resolver) !=
          PARAM_OK ||
      !ParamTable_IsBound()) {
    return 1;
  }

  ParamTable_Init();
  return ParamTableTest_AssertValuesInitialized();
}

static int TestParamTableRejectsInvalidRebindAtomically(void) {
  ParamTableTestResolver invalid_resolver = {
      .mode = PARAM_TABLE_TEST_BINDING_INDEX_MISMATCH};
  if (ParamTable_SetBindingAdapter(ParamTableTest_Resolve, &invalid_resolver) !=
      PARAM_ERR_INVALID_INDEX) {
    return 1;
  }

  invalid_resolver.mode = PARAM_TABLE_TEST_BINDING_TYPE_MISMATCH;
  if (ParamTable_SetBindingAdapter(ParamTableTest_Resolve, &invalid_resolver) !=
      PARAM_ERR_INVALID_TYPE) {
    return 1;
  }

  invalid_resolver.mode = PARAM_TABLE_TEST_BINDING_NULL_TARGET;
  if (ParamTable_SetBindingAdapter(ParamTableTest_Resolve, &invalid_resolver) !=
      PARAM_ERR_NULL_PTR) {
    return 1;
  }

  invalid_resolver.mode = PARAM_TABLE_TEST_BINDING_NAN_DEFAULT;
  if (ParamTable_SetBindingAdapter(ParamTableTest_Resolve, &invalid_resolver) !=
      PARAM_ERR_OUT_OF_RANGE) {
    return 1;
  }

  ParamTable_Init();
  return ParamTableTest_AssertValuesInitialized();
}

int main(void) {
  if (TestParamTableRequiresCompleteBindings()) {
    printf("FAIL parameter table did not require complete bindings\n");
    return 1;
  }
  if (TestParamTableRejectsInvalidRebindAtomically()) {
    printf("FAIL parameter table accepted an invalid binding\n");
    return 1;
  }
  printf("Parameter table binding tests passed\n");
  return 0;
}
