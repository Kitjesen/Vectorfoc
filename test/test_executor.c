#include "executor.h"
#include "fsm.h"
#include "inovxio_protocol.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int tests_passed = 0;

#define TEST(name)                                                            \
  do {                                                                        \
    reset_test_state();                                                       \
    printf("  Testing: %s ... ", #name);                                      \
    test_##name();                                                            \
    printf("PASS\n");                                                         \
    tests_passed++;                                                           \
  } while (0)

StateMachine g_ds402_state_machine = {0};
MOTOR_DATA motor_data = {0};
uint8_t g_can_id = 1;
uint8_t g_can_baudrate = 0;
uint8_t g_protocol_type = 0;
uint32_t g_can_timeout_ms = 0;
uint8_t g_run_mode = 0;

static ProtocolType s_protocol_type = PROTOCOL_INOVXIO;
static MotorState s_last_requested_state = STATE_NOT_READY_TO_SWITCH_ON;
static uint16_t s_last_controlword = 0;
static int s_request_count = 0;
static int s_send_count = 0;
static int s_schedule_save_count = 0;
static int s_protocol_set_count = 0;
static uint32_t s_fault_bits = 0;
static uint16_t s_last_param_index = 0;
static float s_last_param_value = 0.0f;
static CAN_Frame s_last_sent_frame = {0};
static float s_encoder_position = 0.0f;
static float s_encoder_offset = 0.0f;

static ParamEntry s_param_table[] = {
    {.index = PARAM_CAN_ID,
     .type = PARAM_TYPE_UINT8,
     .access = PARAM_ACCESS_RW,
     .ptr = &g_can_id,
     .min = 1.0f,
     .max = 127.0f},
    {.index = PARAM_CAN_BAUDRATE,
     .type = PARAM_TYPE_UINT8,
     .access = PARAM_ACCESS_RW,
     .ptr = &g_can_baudrate,
     .min = 0.0f,
     .max = 2.0f},
    {.index = PARAM_PROTOCOL_TYPE,
     .type = PARAM_TYPE_UINT8,
     .access = PARAM_ACCESS_RW,
     .ptr = &g_protocol_type,
     .min = 0.0f,
     .max = 2.0f},
    {.index = PARAM_CAN_TIMEOUT,
     .type = PARAM_TYPE_UINT32,
     .access = PARAM_ACCESS_RW,
     .ptr = &g_can_timeout_ms,
     .min = 0.0f,
     .max = 10000.0f},
    {.index = PARAM_RUN_MODE,
     .type = PARAM_TYPE_UINT8,
     .access = PARAM_ACCESS_RW,
     .ptr = &g_run_mode,
     .min = 0.0f,
     .max = 10.0f},
};

static void reset_test_state(void) {
  memset(&g_ds402_state_machine, 0, sizeof(g_ds402_state_machine));
  memset(&motor_data, 0, sizeof(motor_data));
  memset(&s_last_sent_frame, 0, sizeof(s_last_sent_frame));
  g_ds402_state_machine.current_state = STATE_OPERATION_ENABLED;
  s_protocol_type = PROTOCOL_INOVXIO;
  s_last_requested_state = STATE_NOT_READY_TO_SWITCH_ON;
  s_last_controlword = 0;
  s_request_count = 0;
  s_send_count = 0;
  s_schedule_save_count = 0;
  s_protocol_set_count = 0;
  s_fault_bits = 0;
  s_last_param_index = 0;
  s_last_param_value = 0.0f;
  s_encoder_position = 1.5f;
  s_encoder_offset = 0.0f;
  g_can_id = 1;
  g_can_baudrate = 0;
  g_protocol_type = 0;
  g_can_timeout_ms = 0;
  g_run_mode = 0;
}

bool StateMachine_RequestState(StateMachine *sm, MotorState target_state) {
  s_request_count++;
  s_last_requested_state = target_state;
  sm->current_state = target_state;
  return true;
}

void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword) {
  s_last_controlword = controlword;
  sm->controlword.word = controlword;
}

MotorState StateMachine_GetState(const StateMachine *sm) {
  return sm->current_state;
}

ProtocolType Protocol_GetType(void) { return s_protocol_type; }

void Protocol_SetType(ProtocolType protocol) {
  s_protocol_set_count++;
  s_protocol_type = protocol;
}

bool ProtocolPrivate_BuildFaultDetail(const MotorStatus *status,
                                      CAN_Frame *frame) {
  frame->id = 0x111;
  frame->dlc = 8;
  frame->data[0] = (uint8_t)(status->fault_code & 0xFF);
  return true;
}

bool Protocol_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  frame->id = 0x222;
  frame->dlc = 4;
  frame->data[0] = (uint8_t)(fault_code & 0xFF);
  return true;
}

bool Protocol_BuildParamResponse(uint16_t param_index, float value,
                                 CAN_Frame *frame) {
  s_last_param_index = param_index;
  s_last_param_value = value;
  frame->id = 0x333;
  frame->dlc = 8;
  return true;
}

bool Protocol_SendFrame(const CAN_Frame *frame) {
  s_last_sent_frame = *frame;
  s_send_count++;
  return true;
}

const ParamEntry *ParamTable_Find(uint16_t index) {
  uint32_t i = 0;

  for (i = 0; i < sizeof(s_param_table) / sizeof(s_param_table[0]); ++i) {
    if (s_param_table[i].index == index) {
      return &s_param_table[i];
    }
  }

  return NULL;
}

uint32_t ParamTable_GetCount(void) {
  return (uint32_t)(sizeof(s_param_table) / sizeof(s_param_table[0]));
}

const ParamEntry *ParamTable_GetTable(void) { return s_param_table; }

ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry) {
  const ParamEntry *found = ParamTable_Find(index);

  if (found == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }

  *entry = found;
  return PARAM_OK;
}

ParamResult Param_Write(uint16_t index, const void *data) {
  const ParamEntry *entry = ParamTable_Find(index);

  if (entry == NULL || data == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }

  switch (entry->type) {
  case PARAM_TYPE_UINT8:
    *(uint8_t *)entry->ptr = *(const uint8_t *)data;
    return PARAM_OK;
  case PARAM_TYPE_UINT16:
    *(uint16_t *)entry->ptr = *(const uint16_t *)data;
    return PARAM_OK;
  case PARAM_TYPE_UINT32:
    *(uint32_t *)entry->ptr = *(const uint32_t *)data;
    return PARAM_OK;
  case PARAM_TYPE_INT32:
    *(int32_t *)entry->ptr = *(const int32_t *)data;
    return PARAM_OK;
  case PARAM_TYPE_FLOAT:
    *(float *)entry->ptr = *(const float *)data;
    return PARAM_OK;
  default:
    return PARAM_ERR_INVALID_TYPE;
  }
}

ParamResult Param_WriteFloat(uint16_t index, float value) {
  const ParamEntry *entry = ParamTable_Find(index);

  if (entry == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }
  if (entry->type != PARAM_TYPE_FLOAT) {
    return PARAM_ERR_INVALID_TYPE;
  }

  *(float *)entry->ptr = value;
  return PARAM_OK;
}

ParamResult Param_ReadFloat(uint16_t index, float *value) {
  const ParamEntry *entry = ParamTable_Find(index);

  if (entry == NULL || value == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }
  if (entry->type != PARAM_TYPE_FLOAT) {
    return PARAM_ERR_INVALID_TYPE;
  }

  *value = *(float *)entry->ptr;
  return PARAM_OK;
}

ParamResult Param_WriteUint8(uint16_t index, uint8_t value) {
  const ParamEntry *entry = ParamTable_Find(index);

  if (entry == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }
  if (entry->type != PARAM_TYPE_UINT8) {
    return PARAM_ERR_INVALID_TYPE;
  }

  *(uint8_t *)entry->ptr = value;
  return PARAM_OK;
}

void Param_ScheduleSave(void) { s_schedule_save_count++; }

uint32_t Safety_GetActiveFaultBits(void) { return s_fault_bits; }

uint32_t Safety_GetLastFaultTime(void) { return 0; }

void Safety_ClearFaults(StateMachine *fsm) { (void)fsm; }

uint32_t HAL_GetSystemTick(void) { return 0; }

void HAL_Delay(uint32_t ms) { (void)ms; }

float MHAL_Encoder_GetPosition(void) { return s_encoder_position; }

int MHAL_Encoder_SetOffset(float offset) {
  s_encoder_offset = offset;
  return 0;
}

static void store_float(uint32_t *dst, float value) {
  memcpy(dst, &value, sizeof(value));
}

static void test_fault_query_does_not_stop_motor(void) {
  MotorCommand cmd = {0};

  cmd.is_fault_query = true;
  s_fault_bits = 0x1234U;

  Executor_ProcessCommand(&cmd);

  assert(s_request_count == 0);
  assert(s_send_count == 1);
  assert(s_last_sent_frame.id == 0x111U);
}

static void test_mit_command_applies_fields_and_enables_motor(void) {
  MotorCommand cmd = {0};

  cmd.has_enable_command = true;
  cmd.enable_motor = true;
  cmd.control_mode = CONTROL_MODE_MIT;
  cmd.pos_setpoint = 1.25f;
  cmd.vel_setpoint = -2.0f;
  cmd.torque_ff = 3.5f;
  cmd.kp = 12.0f;
  cmd.kd = 0.6f;

  Executor_ProcessCommand(&cmd);

  assert(s_last_requested_state == STATE_OPERATION_ENABLED);
  assert(motor_data.state.Control_Mode == CONTROL_MODE_MIT);
  assert(motor_data.Controller.mit_pos_des == 1.25f);
  assert(motor_data.Controller.mit_vel_des == -2.0f);
  assert(motor_data.Controller.input_torque == 3.5f);
  assert(motor_data.Controller.mit_kp == 12.0f);
  assert(motor_data.Controller.mit_kd == 0.6f);
}

static void test_param_write_run_mode_updates_runtime_mode(void) {
  MotorCommand cmd = {0};
  float value = 3.0f;

  cmd.is_param_write = true;
  cmd.param_index = PARAM_RUN_MODE;
  store_float(&cmd.param_value, value);

  Executor_ProcessCommand(&cmd);

  assert(g_run_mode == 3U);
  assert(motor_data.state.Control_Mode == CONTROL_MODE_TORQUE);
}

static void test_param_write_uint8_uses_typed_path_and_schedules_save(void) {
  MotorCommand cmd = {0};
  float value = 2.0f;

  cmd.is_param_write = true;
  cmd.param_index = PARAM_CAN_BAUDRATE;
  store_float(&cmd.param_value, value);

  Executor_ProcessCommand(&cmd);

  assert(g_can_baudrate == 2U);
  assert(s_schedule_save_count == 1);
}

static void test_param_read_uint8_builds_response_with_float_cast(void) {
  MotorCommand cmd = {0};

  g_protocol_type = 2U;
  cmd.is_param_read = true;
  cmd.param_index = PARAM_PROTOCOL_TYPE;

  Executor_ProcessCommand(&cmd);

  assert(s_last_param_index == PARAM_PROTOCOL_TYPE);
  assert(s_last_param_value == 2.0f);
  assert(s_send_count == 1);
  assert(s_last_sent_frame.id == 0x333U);
}

static void test_protocol_switch_updates_runtime_and_persists(void) {
  MotorCommand cmd = {0};

  cmd.is_protocol_switch = true;
  cmd.target_protocol = PROTOCOL_CANOPEN;

  Executor_ProcessCommand(&cmd);

  assert(g_protocol_type == PROTOCOL_CANOPEN);
  assert(s_protocol_type == PROTOCOL_CANOPEN);
  assert(s_protocol_set_count == 1);
  assert(s_schedule_save_count == 1);
}

int main(void) {
  printf("Running executor tests...\n");

  TEST(fault_query_does_not_stop_motor);
  TEST(mit_command_applies_fields_and_enables_motor);
  TEST(param_write_run_mode_updates_runtime_mode);
  TEST(param_write_uint8_uses_typed_path_and_schedules_save);
  TEST(param_read_uint8_builds_response_with_float_cast);
  TEST(protocol_switch_updates_runtime_and_persists);

  printf("All %d executor tests passed.\n", tests_passed);
  return 0;
}
