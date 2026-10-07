#include "parameter_registry.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define MAX_TEST_BINDINGS 48u

typedef union {
  float f;
  uint8_t u8;
  uint16_t u16;
  uint32_t u32;
  int32_t i32;
} TestValue;

typedef enum {
  BIND_VALID,
  BIND_OMIT_LAST,
  BIND_WRONG_INDEX,
  BIND_WRONG_TYPE,
  BIND_NULL_TARGET,
  BIND_NAN_DEFAULT
} BindingMode;

static TestValue values[MAX_TEST_BINDINGS];
static ParamTargetBinding bindings[MAX_TEST_BINDINGS];
static uint32_t binding_count;

static bool resolve_binding(void *context, uint16_t index, ParamType type,
                            ParamTargetBinding *binding) {
  BindingMode mode = *(BindingMode *)context;
  uint32_t available = mode == BIND_OMIT_LAST ? binding_count - 1u
                                               : binding_count;
  for (uint32_t i = 0; i < available; ++i) {
    if (bindings[i].index != index)
      continue;
    *binding = bindings[i];
    if (mode == BIND_WRONG_INDEX)
      binding->index = 0u;
    else if (mode == BIND_WRONG_TYPE)
      binding->type = type == PARAM_TYPE_FLOAT ? PARAM_TYPE_UINT8
                                               : PARAM_TYPE_FLOAT;
    else if (mode == BIND_NULL_TARGET)
      binding->target = NULL;
    else if (mode == BIND_NAN_DEFAULT)
      binding->default_val = NAN;
    return true;
  }
  return false;
}

static void build_valid_bindings(void) {
  const ParamEntry *table = ParamTable_GetTable();
  binding_count = ParamTable_GetCount();
  assert(table != NULL);
  assert(binding_count > 0u && binding_count <= MAX_TEST_BINDINGS);
  memset(values, 0, sizeof(values));
  for (uint32_t i = 0; i < binding_count; ++i) {
    float default_value = table[i].type == PARAM_TYPE_FLOAT
                              ? table[i].min + (table[i].max - table[i].min) * 0.5f
                              : table[i].min;
    bindings[i] = (ParamTargetBinding){.index = table[i].index,
                                       .type = table[i].type,
                                       .target = &values[i],
                                       .default_val = default_value};
  }
}

static void assert_valid_binding_remains_installed(void) {
  const ParamEntry *table = ParamTable_GetTable();
  for (uint32_t i = 0; i < binding_count; ++i) {
    ParamTargetBinding actual = {0};
    assert(ParamTable_GetBinding(&table[i], &actual) == PARAM_OK);
    assert(actual.target == bindings[i].target);
    assert(actual.default_val == bindings[i].default_val);
  }
}

static void requires_complete_bindings(void) {
  BindingMode mode = BIND_OMIT_LAST;
  build_valid_bindings();
  assert(!ParamTable_IsBound());
  assert(ParamTable_SetBindingAdapter(resolve_binding, &mode) ==
         PARAM_ERR_INVALID_INDEX);
  assert(!ParamTable_IsBound());

  mode = BIND_VALID;
  assert(ParamTable_SetBindingAdapter(resolve_binding, &mode) == PARAM_OK);
  assert(ParamTable_IsBound());
  ParamTable_Init();
  assert_valid_binding_remains_installed();
}

static void invalid_rebind_is_atomic(void) {
  const BindingMode invalid_modes[] = {BIND_WRONG_INDEX, BIND_WRONG_TYPE,
                                       BIND_NULL_TARGET, BIND_NAN_DEFAULT};
  const ParamResult errors[] = {PARAM_ERR_INVALID_INDEX, PARAM_ERR_INVALID_TYPE,
                                PARAM_ERR_NULL_PTR, PARAM_ERR_OUT_OF_RANGE};
  for (unsigned i = 0; i < sizeof(invalid_modes) / sizeof(invalid_modes[0]); ++i) {
    BindingMode mode = invalid_modes[i];
    assert(ParamTable_SetBindingAdapter(resolve_binding, &mode) == errors[i]);
    assert(ParamTable_IsBound());
    assert_valid_binding_remains_installed();
  }
}

int main(void) {
  requires_complete_bindings();
  invalid_rebind_is_atomic();
  return 0;
}
