#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "executor.h"
#include "fsm.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "vector_protocol.h"

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      return 1;                                                                \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                \
  CHECK(fabsf((actual) - (expected)) <= (tolerance))

StateMachine g_ds402_state_machine;
MOTOR_DATA motor_data;
uint8_t g_can_id = 1U;
uint8_t g_protocol_type = PROTOCOL_VECTOR;

static MotorState s_state = STATE_OPERATION_ENABLED;
static unsigned s_request_count;
static MotorState s_last_requested_state;
static unsigned s_zero_position_count;
static unsigned s_timeout_write_count;
static uint32_t s_written_timeout_ms;
static unsigned s_timeout_config_count;
static uint32_t s_configured_timeout_ms;
static unsigned s_param_response_count;
static uint16_t s_param_response_index;
static float s_param_response_value;
static unsigned s_param_write_count;
static uint16_t s_param_write_index;
static float s_param_write_value;
static ParamResult s_param_write_result;
static uint8_t s_stored_uint8;
static uint32_t s_stored_uint32;
static unsigned s_protocol_set_count;
static ProtocolType s_protocol_set_value;
static unsigned s_run_mode_apply_count;
static unsigned s_save_count;
static ParamEntry s_param_entry;

static void reset_fixture(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  s_state = STATE_OPERATION_ENABLED;
  s_request_count = 0U;
  s_zero_position_count = 0U;
  s_timeout_write_count = 0U;
  s_written_timeout_ms = 0U;
  s_timeout_config_count = 0U;
  s_configured_timeout_ms = 0U;
  s_param_response_count = 0U;
  s_param_response_index = 0U;
  s_param_response_value = 0.0f;
  s_last_requested_state = STATE_OPERATION_ENABLED;
  s_param_write_count = 0U;
  s_param_write_index = 0U;
  s_param_write_value = 0.0f;
  s_param_write_result = PARAM_OK;
  s_stored_uint8 = 0U;
  s_stored_uint32 = 333U;
  s_protocol_set_count = 0U;
  s_protocol_set_value = PROTOCOL_VECTOR;
  s_run_mode_apply_count = 0U;
  s_save_count = 0U;
  memset(&s_param_entry, 0, sizeof(s_param_entry));
  s_param_entry.need_save = true;
  g_protocol_type = PROTOCOL_VECTOR;
}

static int
test_position_and_velocity_commands_convert_si_to_internal_units(void) {
  CHECK_NEAR(Protocol_TurnsToRadians(1.0f), 2.0f * (float)M_PI, 1e-5f);
  CHECK_NEAR(Protocol_RadiansToTurns((float)M_PI), 0.5f, 1e-5f);
  reset_fixture();
  MotorCommand cmd = {0};
  cmd.control_mode = CONTROL_MODE_POSITION;
  cmd.position_ref = 2.0f * (float)M_PI;
  Executor_ProcessCommand(&cmd);
  CHECK_NEAR(motor_data.Controller.input_position, 1.0f, 1e-5f);

  memset(&cmd, 0, sizeof(cmd));
  cmd.control_mode = CONTROL_MODE_VELOCITY;
  cmd.speed_ref = (float)M_PI;
  Executor_ProcessCommand(&cmd);
  CHECK_NEAR(motor_data.Controller.input_velocity, 0.5f, 1e-5f);
  return 0;
}

static int test_non_motion_command_does_not_change_motor_state(void) {
  reset_fixture();
  MotorCommand cmd = {0};
  cmd.is_fault_query = true;
  Executor_ProcessCommand(&cmd);
  CHECK(s_request_count == 0U);
  CHECK(s_state == STATE_OPERATION_ENABLED);
  return 0;
}

static int test_explicit_stop_requests_disabled_state(void) {
  reset_fixture();
  MotorCommand cmd = {0};
  cmd.has_enable_command = true;
  cmd.enable_motor = false;
  Executor_ProcessCommand(&cmd);
  CHECK(s_request_count == 1U);
  CHECK(s_last_requested_state == STATE_SWITCH_ON_DISABLED);
  return 0;
}

static int test_set_zero_resets_only_the_mechanical_position_origin(void) {
  reset_fixture();
  motor_data.feedback.position = 3.5f;
  motor_data.Controller.input_position = 3.5f;
  motor_data.Controller.pos_setpoint = 3.5f;
  motor_data.Controller.mit_pos_des = 7.0f;

  MotorCommand cmd = {0};
  cmd.set_zero = true;
  Executor_ProcessCommand(&cmd);

  CHECK(s_zero_position_count == 1U);
  CHECK_NEAR(motor_data.feedback.position, 0.0f, 1e-6f);
  CHECK_NEAR(motor_data.Controller.input_position, 0.0f, 1e-6f);
  CHECK_NEAR(motor_data.Controller.pos_setpoint, 0.0f, 1e-6f);
  CHECK_NEAR(motor_data.Controller.mit_pos_des, 0.0f, 1e-6f);
  return 0;
}

static int test_explicit_open_control_mode_is_not_treated_as_absent(void) {
  reset_fixture();
  motor_data.state.Control_Mode = CONTROL_MODE_POSITION;

  MotorCommand cmd = {0};
  cmd.has_control_mode = true;
  cmd.control_mode = CONTROL_MODE_OPEN;
  Executor_ProcessCommand(&cmd);

  CHECK(motor_data.state.Control_Mode == CONTROL_MODE_OPEN);
  return 0;
}

static int
test_explicit_mode_change_keeps_omitted_motion_setpoints_unchanged(void) {
  reset_fixture();
  motor_data.Controller.input_position = 1.25f;
  motor_data.Controller.input_velocity = -2.5f;
  motor_data.Controller.input_torque = 3.75f;

  const CONTROL_MODE modes[] = {CONTROL_MODE_POSITION,
                                CONTROL_MODE_VELOCITY,
                                CONTROL_MODE_TORQUE};
  for (size_t i = 0U; i < sizeof(modes) / sizeof(modes[0]); ++i) {
    MotorCommand cmd = {0};
    cmd.has_control_mode = true;
    cmd.control_mode = (uint8_t)modes[i];
    Executor_ProcessCommand(&cmd);

    CHECK(motor_data.state.Control_Mode == modes[i]);
    CHECK_NEAR(motor_data.Controller.input_position, 1.25f, 1e-6f);
    CHECK_NEAR(motor_data.Controller.input_velocity, -2.5f, 1e-6f);
    CHECK_NEAR(motor_data.Controller.input_torque, 3.75f, 1e-6f);
  }
  return 0;
}

static int test_current_references_publish_through_executor(void) {
  reset_fixture();

  MotorCommand cmd = {0};
  cmd.has_iq_ref = true;
  cmd.iq_ref = 1.25f;
  cmd.has_id_ref = true;
  cmd.id_ref = -0.5f;
  Executor_ProcessCommand(&cmd);

  CHECK_NEAR(motor_data.algo_input.Iq_ref, 1.25f, 1e-6f);
  CHECK_NEAR(motor_data.algo_input.Id_ref, -0.5f, 1e-6f);
  return 0;
}

static int test_can_timeout_parameter_updates_runtime_detection(void) {
  reset_fixture();
  MotorCommand cmd = {0};
  float timeout_ms = 250.0f;
  cmd.is_param_write = true;
  cmd.param_index = PARAM_CAN_TIMEOUT;
  memcpy(&cmd.param_value, &timeout_ms, sizeof(timeout_ms));

  Executor_ProcessCommand(&cmd);

  CHECK(s_timeout_write_count == 1U);
  CHECK(s_written_timeout_ms == 250U);
  CHECK(s_timeout_config_count == 1U);
  CHECK(s_configured_timeout_ms == 250U);
  return 0;
}

static int test_can_timeout_parameter_read_uses_uint32_accessor(void) {
  reset_fixture();
  MotorCommand cmd = {0};
  cmd.is_param_read = true;
  cmd.param_index = PARAM_CAN_TIMEOUT;

  Executor_ProcessCommand(&cmd);

  CHECK(s_param_response_count == 1U);
  CHECK(s_param_response_index == PARAM_CAN_TIMEOUT);
  CHECK_NEAR(s_param_response_value, 333.0f, 1e-6f);
  return 0;
}

static int
test_failed_protocol_switch_has_no_runtime_or_save_side_effect(void) {
  reset_fixture();
  s_param_write_result = PARAM_ERR_STORAGE;
  MotorCommand cmd = {0};
  cmd.is_protocol_switch = true;
  cmd.target_protocol = PROTOCOL_MIT;

  Executor_ProcessCommand(&cmd);

  CHECK(s_param_write_count == 1U);
  CHECK(s_param_write_index == PARAM_PROTOCOL_TYPE);
  CHECK(s_protocol_set_count == 0U);
  CHECK(s_save_count == 0U);
  CHECK(g_protocol_type == PROTOCOL_VECTOR);
  return 0;
}

static int test_successful_protocol_switch_persists_then_applies(void) {
  reset_fixture();
  MotorCommand cmd = {0};
  cmd.is_protocol_switch = true;
  cmd.target_protocol = PROTOCOL_CANOPEN;

  Executor_ProcessCommand(&cmd);

  CHECK(s_param_write_count == 1U);
  CHECK(s_param_write_index == PARAM_PROTOCOL_TYPE);
  CHECK(s_stored_uint8 == PROTOCOL_CANOPEN);
  CHECK(s_protocol_set_count == 1U);
  CHECK(s_protocol_set_value == PROTOCOL_CANOPEN);
  CHECK(s_save_count == 1U);
  return 0;
}

static int
test_failed_parameter_write_has_no_runtime_or_save_side_effect(void) {
  reset_fixture();
  s_param_write_result = PARAM_ERR_OUT_OF_RANGE;
  MotorCommand cmd = {0};
  float timeout_ms = 250.5f;
  cmd.is_param_write = true;
  cmd.param_index = PARAM_CAN_TIMEOUT;
  memcpy(&cmd.param_value, &timeout_ms, sizeof(timeout_ms));

  Executor_ProcessCommand(&cmd);

  CHECK(s_param_write_count == 1U);
  CHECK(s_timeout_config_count == 0U);
  CHECK(s_save_count == 0U);
  return 0;
}

static int test_run_mode_write_updates_runtime_mode_after_success(void) {
  reset_fixture();
  MotorCommand cmd = {0};
  float run_mode = 5.0f;
  cmd.is_param_write = true;
  cmd.param_index = PARAM_RUN_MODE;
  memcpy(&cmd.param_value, &run_mode, sizeof(run_mode));

  Executor_ProcessCommand(&cmd);

  CHECK(s_stored_uint8 == 5U);
  CHECK(motor_data.state.Control_Mode == CONTROL_MODE_POSITION);
  CHECK(s_run_mode_apply_count == 1U);
  CHECK(s_save_count == 1U);
  return 0;
}

int main(void) {
  int failures = 0;
  failures +=
      test_position_and_velocity_commands_convert_si_to_internal_units();
  failures += test_non_motion_command_does_not_change_motor_state();
  failures += test_explicit_stop_requests_disabled_state();
  failures += test_set_zero_resets_only_the_mechanical_position_origin();
  failures += test_explicit_open_control_mode_is_not_treated_as_absent();
  failures +=
      test_explicit_mode_change_keeps_omitted_motion_setpoints_unchanged();
  failures += test_current_references_publish_through_executor();
  failures += test_can_timeout_parameter_updates_runtime_detection();
  failures += test_can_timeout_parameter_read_uses_uint32_accessor();
  failures += test_failed_protocol_switch_has_no_runtime_or_save_side_effect();
  failures += test_successful_protocol_switch_persists_then_applies();
  failures += test_failed_parameter_write_has_no_runtime_or_save_side_effect();
  failures += test_run_mode_write_updates_runtime_mode_after_success();
  if (failures == 0) {
    printf("All communication executor tests PASSED\n");
    return 0;
  }
  return 1;
}

bool StateMachine_RequestState(StateMachine *sm, MotorState target_state) {
  (void)sm;
  s_request_count++;
  s_last_requested_state = target_state;
  s_state = target_state;
  return true;
}

void StateMachine_SetControlword(StateMachine *sm, uint16_t controlword) {
  (void)sm;
  (void)controlword;
}

MotorState StateMachine_GetState(const StateMachine *sm) {
  (void)sm;
  return s_state;
}

uint32_t Safety_GetActiveFaultBits(void) { return 0U; }
bool Motor_ClearFaults(MOTOR_DATA *motor) {
  (void)motor;
  return true;
}
int MHAL_Encoder_ZeroPosition(void) {
  s_zero_position_count++;
  return 0;
}

ProtocolType Protocol_GetType(void) { return PROTOCOL_VECTOR; }
void Protocol_SetType(ProtocolType protocol) {
  s_protocol_set_count++;
  s_protocol_set_value = protocol;
  g_protocol_type = (uint8_t)protocol;
}
bool Protocol_SendFrame(const CAN_Frame *frame) {
  (void)frame;
  return true;
}
bool Protocol_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  (void)fault_code;
  memset(frame, 0, sizeof(*frame));
  return true;
}
bool Protocol_BuildParamResponse(uint16_t param_index, float value,
                                 CAN_Frame *frame) {
  s_param_response_count++;
  s_param_response_index = param_index;
  s_param_response_value = value;
  memset(frame, 0, sizeof(*frame));
  return true;
}
bool ProtocolVector_BuildFaultDetail(const MotorStatus *status,
                                     CAN_Frame *frame) {
  (void)status;
  memset(frame, 0, sizeof(*frame));
  return true;
}

ParamResult Param_ReadFloat(uint16_t index, float *value) {
  (void)index;
  if (value != NULL) {
    *value = 0.0f;
  }
  return PARAM_OK;
}
ParamResult Param_ReadUint8(uint16_t index, uint8_t *value) {
  if (value == NULL) {
    return PARAM_ERR_NULL_PTR;
  }
  if (index != PARAM_RUN_MODE && index != PARAM_PROTOCOL_TYPE &&
      index != PARAM_CAN_ID) {
    return PARAM_ERR_INVALID_INDEX;
  }
  *value = s_stored_uint8;
  return PARAM_OK;
}
ParamResult Param_ReadUint32(uint16_t index, uint32_t *value) {
  if (index != PARAM_CAN_TIMEOUT || value == NULL) {
    return PARAM_ERR_INVALID_INDEX;
  }
  *value = s_stored_uint32;
  return PARAM_OK;
}
ParamResult Param_ReadAsFloat(uint16_t index, float *value) {
  if (value == NULL) {
    return PARAM_ERR_NULL_PTR;
  }
  if (index == PARAM_CAN_TIMEOUT) {
    *value = (float)s_stored_uint32;
    return PARAM_OK;
  }
  if (index == PARAM_RUN_MODE || index == PARAM_PROTOCOL_TYPE ||
      index == PARAM_CAN_ID) {
    *value = (float)s_stored_uint8;
    return PARAM_OK;
  }
  return PARAM_ERR_INVALID_INDEX;
}
ParamResult Param_WriteFloat(uint16_t index, float value) {
  (void)index;
  (void)value;
  return PARAM_OK;
}
ParamResult Param_WriteUint8(uint16_t index, uint8_t value) {
  s_param_write_count++;
  s_param_write_index = index;
  s_param_write_value = (float)value;
  if (s_param_write_result != PARAM_OK) {
    return s_param_write_result;
  }
  s_stored_uint8 = value;
  if (index == PARAM_PROTOCOL_TYPE) {
    Protocol_SetType((ProtocolType)value);
  } else if (index == PARAM_RUN_MODE) {
    CONTROL_MODE mode;
    if (Motor_RunModeToControlMode(value, &mode)) {
      motor_data.state.Control_Mode = mode;
      s_run_mode_apply_count++;
    }
  }
  return PARAM_OK;
}
void Detection_SetCANTimeout(uint32_t timeout_ms);

ParamResult Param_WriteUint32(uint16_t index, uint32_t value) {
  if (index != PARAM_CAN_TIMEOUT) {
    return PARAM_ERR_INVALID_INDEX;
  }
  if (s_param_write_result != PARAM_OK) {
    return s_param_write_result;
  }
  s_timeout_write_count++;
  s_written_timeout_ms = value;
  s_stored_uint32 = value;
  return PARAM_OK;
}
ParamResult Param_WriteFromFloat(uint16_t index, float value) {
  s_param_write_count++;
  s_param_write_index = index;
  s_param_write_value = value;
  if (s_param_write_result != PARAM_OK) {
    return s_param_write_result;
  }
  if (index == PARAM_CAN_TIMEOUT) {
    if (!isfinite(value) || value < 0.0f || value != floorf(value)) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    s_timeout_write_count++;
    s_written_timeout_ms = (uint32_t)value;
    s_stored_uint32 = (uint32_t)value;
    Detection_SetCANTimeout(s_stored_uint32);
    return PARAM_OK;
  }
  if (index == PARAM_RUN_MODE || index == PARAM_PROTOCOL_TYPE ||
      index == PARAM_CAN_ID) {
    if (!isfinite(value) || value < 0.0f || value > 255.0f ||
        value != floorf(value)) {
      return PARAM_ERR_OUT_OF_RANGE;
    }
    s_stored_uint8 = (uint8_t)value;
    if (index == PARAM_PROTOCOL_TYPE) {
      Protocol_SetType((ProtocolType)s_stored_uint8);
    } else if (index == PARAM_RUN_MODE) {
      CONTROL_MODE mode;
      if (Motor_RunModeToControlMode(s_stored_uint8, &mode)) {
        motor_data.state.Control_Mode = mode;
        s_run_mode_apply_count++;
      }
    }
    return PARAM_OK;
  }
  return PARAM_OK;
}
ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry) {
  (void)index;
  if (entry == NULL) {
    return PARAM_ERR_NULL_PTR;
  }
  *entry = &s_param_entry;
  return PARAM_OK;
}
void Detection_SetCANTimeout(uint32_t timeout_ms) {
  s_timeout_config_count++;
  s_configured_timeout_ms = timeout_ms;
}
void Param_ScheduleSave(void) { s_save_count++; }
