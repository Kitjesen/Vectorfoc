#include "inovxio_protocol.h"
#include "fault_def.h"
#include "fsm.h"
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define EPSILON 1e-3f
#define ASSERT_FLOAT_NEAR(a, b, eps) assert(fabsf((a) - (b)) < (eps))

static int tests_passed = 0;

#define TEST(name)                                                            \
  do {                                                                        \
    reset_test_state();                                                       \
    printf("  Testing: %s ... ", #name);                                      \
    test_##name();                                                            \
    printf("PASS\n");                                                         \
    tests_passed++;                                                           \
  } while (0)

uint8_t g_can_id = 7;
uint8_t g_can_baudrate = 1;
uint8_t g_protocol_type = 0;
uint32_t g_can_timeout_ms = 0;
uint8_t g_run_mode = 0;
MOTOR_DATA motor_data = {0};
StateMachine g_ds402_state_machine = {0};

static CAN_Frame s_last_sent_frame = {0};
static int s_send_count = 0;
static int s_schedule_save_count = 0;
static int s_reset_count = 0;
static int s_bootloader_count = 0;
static int s_safety_clear_count = 0;
static int s_motor_clear_count = 0;
static int s_calibration_request_count = 0;
static uint8_t s_last_calibration_type = 0;
static bool s_report_enabled = false;
static bool s_report_called = false;
static uint16_t s_last_param_index = 0;
static uint8_t s_last_param_u8 = 0;
static uint32_t s_last_fault_time = 0x12345678U;

static uint16_t float_to_uint16(float x, float min_val, float max_val) {
  float span = max_val - min_val;
  float scaled = (x - min_val) / span;

  if (scaled < 0.0f) {
    scaled = 0.0f;
  }
  if (scaled > 1.0f) {
    scaled = 1.0f;
  }

  return (uint16_t)(scaled * 65535.0f);
}

static uint32_t make_private_id(uint8_t cmd, uint16_t info, uint8_t target) {
  return ((uint32_t)cmd << 24) | ((uint32_t)info << 8) | target;
}

static void reset_test_state(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  memset(&g_ds402_state_machine, 0, sizeof(g_ds402_state_machine));
  memset(&s_last_sent_frame, 0, sizeof(s_last_sent_frame));
  s_send_count = 0;
  s_schedule_save_count = 0;
  s_reset_count = 0;
  s_bootloader_count = 0;
  s_safety_clear_count = 0;
  s_motor_clear_count = 0;
  s_calibration_request_count = 0;
  s_last_calibration_type = 0;
  s_report_enabled = false;
  s_report_called = false;
  s_last_param_index = 0;
  s_last_param_u8 = 0;
  s_last_fault_time = 0x12345678U;
  g_can_id = 7;
  g_can_baudrate = 1;
}

void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t calib_type) {
  (void)motor;
  s_calibration_request_count++;
  s_last_calibration_type = calib_type;
}

void Motor_ClearFaults(MOTOR_DATA *motor) {
  (void)motor;
  s_motor_clear_count++;
}

ParamResult Param_WriteUint8(uint16_t index, uint8_t value) {
  s_last_param_index = index;
  s_last_param_u8 = value;

  if (index == PARAM_CAN_ID) {
    g_can_id = value;
  } else if (index == PARAM_CAN_BAUDRATE) {
    g_can_baudrate = value;
  }

  return PARAM_OK;
}

ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry) {
  (void)index;
  (void)entry;
  return PARAM_ERR_INVALID_INDEX;
}

ParamResult Param_Write(uint16_t index, const void *data) {
  (void)index;
  (void)data;
  return PARAM_ERR_INVALID_INDEX;
}

ParamResult Param_WriteFloat(uint16_t index, float value) {
  (void)index;
  (void)value;
  return PARAM_ERR_INVALID_INDEX;
}

ParamResult Param_ReadFloat(uint16_t index, float *value) {
  (void)index;
  (void)value;
  return PARAM_ERR_INVALID_INDEX;
}

void Param_ScheduleSave(void) { s_schedule_save_count++; }

bool Protocol_SendFrame(const CAN_Frame *frame) {
  s_last_sent_frame = *frame;
  s_send_count++;
  return true;
}

void CmdService_SetReportEnable(bool enable) {
  s_report_called = true;
  s_report_enabled = enable;
}

void HAL_NVIC_SystemReset(void) { s_reset_count++; }

void Safety_ClearFaults(StateMachine *fsm) {
  (void)fsm;
  s_safety_clear_count++;
}

uint32_t Safety_GetActiveFaultBits(void) { return 0; }

uint32_t Safety_GetLastFaultTime(void) { return s_last_fault_time; }

void Boot_RequestUpgrade(void) { s_bootloader_count++; }

static void test_private_parse_rejects_non_extended_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_MOTOR_ENABLE, 0, g_can_id);
  frame.dlc = 0;
  frame.is_extended = false;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
}

static void test_motor_ctrl_parses_all_fields(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  uint16_t torque_raw = float_to_uint16(15.0f, -120.0f, 120.0f);
  uint16_t pos_raw = float_to_uint16(1.2f, -12.57f, 12.57f);
  uint16_t vel_raw = float_to_uint16(-2.5f, -15.0f, 15.0f);
  uint16_t kp_raw = float_to_uint16(120.0f, 0.0f, 500.0f);
  uint16_t kd_raw = float_to_uint16(3.0f, 0.0f, 100.0f);

  frame.id = make_private_id(PRIVATE_CMD_MOTOR_CTRL, torque_raw, g_can_id);
  frame.is_extended = true;
  frame.dlc = 8;
  frame.data[0] = (uint8_t)(pos_raw >> 8);
  frame.data[1] = (uint8_t)(pos_raw & 0xFF);
  frame.data[2] = (uint8_t)(vel_raw >> 8);
  frame.data[3] = (uint8_t)(vel_raw & 0xFF);
  frame.data[4] = (uint8_t)(kp_raw >> 8);
  frame.data[5] = (uint8_t)(kp_raw & 0xFF);
  frame.data[6] = (uint8_t)(kd_raw >> 8);
  frame.data[7] = (uint8_t)(kd_raw & 0xFF);

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_MIT);
  assert(cmd.has_enable_command);
  assert(cmd.enable_motor);
  ASSERT_FLOAT_NEAR(cmd.torque_ff, 15.0f, 0.02f);
  ASSERT_FLOAT_NEAR(cmd.pos_setpoint, 1.2f, 0.02f);
  ASSERT_FLOAT_NEAR(cmd.vel_setpoint, -2.5f, 0.02f);
  ASSERT_FLOAT_NEAR(cmd.kp, 120.0f, 0.1f);
  ASSERT_FLOAT_NEAR(cmd.kd, 3.0f, 0.02f);
}

static void test_motor_enable_sets_enable_flag(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_MOTOR_ENABLE, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.has_enable_command);
  assert(cmd.enable_motor);
}

static void test_motor_stop_sets_disable_flag(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_MOTOR_STOP, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.has_enable_command);
  assert(!cmd.enable_motor);
}

static void test_set_zero_sets_flag(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_ZERO, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.set_zero);
}

static void test_calibrate_defaults_to_type_zero(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_CALIBRATE, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 0;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_calibration_request_count == 1);
  assert(s_last_calibration_type == 0U);
}

static void test_calibrate_uses_payload_type(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_CALIBRATE, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 1;
  frame.data[0] = 3U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_calibration_request_count == 1);
  assert(s_last_calibration_type == 3U);
}

static void test_set_id_writes_can_id_and_schedules_save(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_ID, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 4;
  frame.data[0] = 11U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_last_param_index == PARAM_CAN_ID);
  assert(s_last_param_u8 == 11U);
  assert(g_can_id == 11U);
  assert(s_schedule_save_count == 1);
}

static void test_set_id_rejects_short_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_ID, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 3;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
  assert(s_schedule_save_count == 0);
}

static void test_param_read_accepts_two_byte_payload(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_PARAM_READ, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 2;
  frame.data[0] = 0x34U;
  frame.data[1] = 0x12U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.is_param_read);
  assert(cmd.param_index == 0x1234U);
}

static void test_param_read_rejects_short_payload(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_PARAM_READ, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 1;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
}

static void test_param_write_reads_index_and_value(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  float value = 42.5f;

  frame.id = make_private_id(PRIVATE_CMD_PARAM_WRITE, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 8;
  frame.data[0] = 0x21U;
  frame.data[1] = 0x43U;
  memcpy(&frame.data[4], &value, sizeof(value));

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.is_param_write);
  assert(cmd.param_index == 0x4321U);
  assert(memcmp(&cmd.param_value, &value, sizeof(value)) == 0);
}

static void test_param_write_rejects_short_payload(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_PARAM_WRITE, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 7;
  frame.data[0] = 0x21U;
  frame.data[1] = 0x43U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
}

static void test_save_schedules_flash_write(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SAVE, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_schedule_save_count == 1);
}

static void test_get_version_sends_response_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_GET_VERSION, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_send_count == 1);
  assert(s_last_sent_frame.id ==
         make_private_id(PRIVATE_CMD_GET_VERSION, g_can_id, 0xFDU));
  assert(s_last_sent_frame.is_extended);
  assert(s_last_sent_frame.dlc == 8);
  assert(s_last_sent_frame.data[0] == 9U);
  assert(s_last_sent_frame.data[1] == 8U);
  assert(s_last_sent_frame.data[2] == 7U);
  assert(s_last_sent_frame.data[3] == 1U);
  assert(s_last_sent_frame.data[4] == (uint8_t)'a');
  assert(s_last_sent_frame.data[5] == (uint8_t)'b');
  assert(s_last_sent_frame.data[6] == (uint8_t)'c');
  assert(s_last_sent_frame.data[7] == (uint8_t)'d');
}

static void test_fault_query_sets_query_flag(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_FAULT_QUERY, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.is_fault_query);
}

static void test_reset_calls_hal_reset(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_RESET, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_reset_count == 1);
}

static void test_clear_fault_clears_safety_and_motor(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_CLEAR_FAULT, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_safety_clear_count == 1);
  assert(s_motor_clear_count == 1);
}

static void test_bootloader_requests_upgrade(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_BOOTLOADER, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_bootloader_count == 1);
}

static void test_report_updates_service_flag(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_REPORT, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 1;
  frame.data[0] = 1U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_report_called);
  assert(s_report_enabled);
}

static void test_report_rejects_short_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_REPORT, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 0;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
  assert(!s_report_called);
}

static void test_set_baudrate_updates_param_and_schedules_save(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_BAUDRATE, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 1;
  frame.data[0] = 2U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(s_last_param_index == PARAM_CAN_BAUDRATE);
  assert(s_last_param_u8 == 2U);
  assert(g_can_baudrate == 2U);
  assert(s_schedule_save_count == 1);
}

static void test_set_baudrate_rejects_short_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_BAUDRATE, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 0;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
}

static void test_set_protocol_marks_switch_request(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_PROTOCOL, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 1;
  frame.data[0] = 2U;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.is_protocol_switch);
  assert(cmd.target_protocol == 2U);
}

static void test_set_protocol_rejects_short_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(PRIVATE_CMD_SET_PROTOCOL, 0, g_can_id);
  frame.is_extended = true;
  frame.dlc = 0;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_INVALID_FRAME);
}

static void test_unknown_command_returns_unsupported(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.id = make_private_id(31U, 0, g_can_id);
  frame.is_extended = true;

  assert(ProtocolPrivate_Parse(&frame, &cmd) == PARSE_ERR_UNSUPPORTED);
}

static void test_build_feedback_encodes_status(void) {
  MotorStatus status = {0};
  CAN_Frame frame = {0};
  uint32_t expected_id = 0;
  uint16_t expected_pos = 0;
  uint16_t expected_vel = 0;
  uint16_t expected_tor = 0;

  status.position = 1.0f;
  status.velocity = -2.0f;
  status.torque = 3.0f;
  status.temperature = 25.5f;
  status.can_id = 9U;
  status.fault_code = FAULT_ENCODER_UNCALIBRATED | FAULT_STALL_OVERLOAD |
                      FAULT_HARDWARE_ID | FAULT_OVER_TEMP | FAULT_CURRENT_A |
                      FAULT_UNDER_VOLTAGE;

  assert(ProtocolPrivate_BuildFeedback(&status, &frame));

  expected_id =
      (0x02U << 24) | (2U << 22) | (0x3FU << 16) | ((uint32_t)9U << 8) | 0xFDU;
  expected_pos = float_to_uint16(status.position, -12.57f, 12.57f);
  expected_vel = float_to_uint16(status.velocity, -15.0f, 15.0f);
  expected_tor = float_to_uint16(status.torque, -120.0f, 120.0f);

  assert(frame.id == expected_id);
  assert(frame.is_extended);
  assert(frame.dlc == 8);
  assert(frame.data[0] == (uint8_t)(expected_pos >> 8));
  assert(frame.data[1] == (uint8_t)(expected_pos & 0xFF));
  assert(frame.data[2] == (uint8_t)(expected_vel >> 8));
  assert(frame.data[3] == (uint8_t)(expected_vel & 0xFF));
  assert(frame.data[4] == (uint8_t)(expected_tor >> 8));
  assert(frame.data[5] == (uint8_t)(expected_tor & 0xFF));
  assert(frame.data[6] == 0x00U);
  assert(frame.data[7] == 0xFFU);
}

static void test_build_fault_encodes_fault_code(void) {
  CAN_Frame frame = {0};

  assert(ProtocolPrivate_BuildFault(0x12345678U, 0, &frame));
  assert(frame.id == make_private_id(PRIVATE_CMD_FAULT, g_can_id, 0xFDU));
  assert(frame.is_extended);
  assert(frame.dlc == 8);
  assert(frame.data[0] == 0x12U);
  assert(frame.data[1] == 0x34U);
  assert(frame.data[2] == 0x56U);
  assert(frame.data[3] == 0x78U);
  assert(frame.data[4] == 0x00U);
  assert(frame.data[5] == 0x00U);
  assert(frame.data[6] == 0x00U);
  assert(frame.data[7] == 0x00U);
}

static void test_build_param_response_encodes_payload(void) {
  CAN_Frame frame = {0};
  float value = 6.25f;

  assert(ProtocolPrivate_BuildParamResponse(0x3456U, value, &frame));
  assert(frame.id == make_private_id(PRIVATE_CMD_PARAM_WRITE, g_can_id, 0xFDU));
  assert(frame.is_extended);
  assert(frame.dlc == 8);
  assert(frame.data[0] == 0x56U);
  assert(frame.data[1] == 0x34U);
  assert(frame.data[2] == 0U);
  assert(frame.data[3] == 0U);
  assert(memcmp(&frame.data[4], &value, sizeof(value)) == 0);
}

static void test_build_fault_detail_uses_timestamp(void) {
  MotorStatus status = {0};
  CAN_Frame frame = {0};

  status.can_id = 4U;
  status.fault_code = 0xA1B2C3D4U;
  s_last_fault_time = 0x89ABCDEFU;

  assert(ProtocolPrivate_BuildFaultDetail(&status, &frame));
  assert(frame.id == make_private_id(PRIVATE_CMD_FAULT_QUERY, 4U, 0xFDU));
  assert(frame.is_extended);
  assert(frame.dlc == 8);
  assert(frame.data[0] == 0xC3U);
  assert(frame.data[1] == 0xD4U);
  assert(frame.data[2] == 0xA1U);
  assert(frame.data[3] == 0xB2U);
  assert(frame.data[4] == 0x89U);
  assert(frame.data[5] == 0xABU);
  assert(frame.data[6] == 0xCDU);
  assert(frame.data[7] == 0xEFU);
}

int main(void) {
  printf("Running Inovxio protocol tests...\n");

  TEST(private_parse_rejects_non_extended_frame);
  TEST(motor_ctrl_parses_all_fields);
  TEST(motor_enable_sets_enable_flag);
  TEST(motor_stop_sets_disable_flag);
  TEST(set_zero_sets_flag);
  TEST(calibrate_defaults_to_type_zero);
  TEST(calibrate_uses_payload_type);
  TEST(set_id_writes_can_id_and_schedules_save);
  TEST(set_id_rejects_short_frame);
  TEST(param_read_accepts_two_byte_payload);
  TEST(param_read_rejects_short_payload);
  TEST(param_write_reads_index_and_value);
  TEST(param_write_rejects_short_payload);
  TEST(save_schedules_flash_write);
  TEST(get_version_sends_response_frame);
  TEST(fault_query_sets_query_flag);
  TEST(reset_calls_hal_reset);
  TEST(clear_fault_clears_safety_and_motor);
  TEST(bootloader_requests_upgrade);
  TEST(report_updates_service_flag);
  TEST(report_rejects_short_frame);
  TEST(set_baudrate_updates_param_and_schedules_save);
  TEST(set_baudrate_rejects_short_frame);
  TEST(set_protocol_marks_switch_request);
  TEST(set_protocol_rejects_short_frame);
  TEST(unknown_command_returns_unsupported);
  TEST(build_feedback_encodes_status);
  TEST(build_fault_encodes_fault_code);
  TEST(build_param_response_encodes_payload);
  TEST(build_fault_detail_uses_timestamp);

  printf("All %d Inovxio protocol tests passed.\n", tests_passed);
  return 0;
}
