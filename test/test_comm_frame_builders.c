#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "protocol/canopen/canopen_protocol.h"
#include "protocol/mit/mit_protocol.h"

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      return 1;                                                                \
    }                                                                          \
  } while (0)

uint8_t g_can_id = 7U;
MOTOR_DATA motor_data;
static CAN_Frame s_last_sent_frame;
static unsigned s_sent_frame_count;

static int32_t read_i32_le(const uint8_t *data) {
  return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

static uint32_t read_u32_le(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void write_i32_le(uint8_t *data, int32_t value) {
  uint32_t raw = (uint32_t)value;
  data[0] = (uint8_t)raw;
  data[1] = (uint8_t)(raw >> 8);
  data[2] = (uint8_t)(raw >> 16);
  data[3] = (uint8_t)(raw >> 24);
}

static bool bytes_are_zero(const uint8_t *data, unsigned start, unsigned end) {
  for (unsigned i = start; i < end; ++i) {
    if (data[i] != 0U) {
      return false;
    }
  }
  return true;
}

static CAN_Frame make_canopen_sdo(uint8_t command, uint16_t index,
                                  uint8_t subindex) {
  CAN_Frame frame = {0};
  frame.id = 0x600U + g_can_id;
  frame.dlc = 8U;
  frame.data[0] = command;
  frame.data[1] = (uint8_t)index;
  frame.data[2] = (uint8_t)(index >> 8);
  frame.data[3] = subindex;
  return frame;
}

static int test_mit_builders_initialize_frame_metadata_and_unused_bytes(void) {
  MotorStatus status = {0};
  CAN_Frame frame;
  memset(&frame, 0xA5, sizeof(frame));
  CHECK(ProtocolMIT_BuildFeedback(&status, &frame));
  CHECK(frame.dlc == 6U);
  CHECK(frame.is_extended == false);
  CHECK(frame.is_rtr == false);
  CHECK(bytes_are_zero(frame.data, 6U, 8U));

  memset(&frame, 0xA5, sizeof(frame));
  CHECK(ProtocolMIT_BuildFault(0U, &frame));
  CHECK(frame.dlc == 4U);
  CHECK(frame.is_extended == false);
  CHECK(frame.is_rtr == false);
  CHECK(bytes_are_zero(frame.data, 4U, 8U));

  CAN_Frame command = {0};
  MotorCommand parsed = {0};
  command.id = g_can_id + 1U;
  command.dlc = 8U;
  CHECK(ProtocolMIT_Parse(&command, &parsed) == PARSE_UNKNOWN_ID);
  command.id = g_can_id;
  command.is_extended = true;
  CHECK(ProtocolMIT_Parse(&command, &parsed) == PARSE_UNKNOWN_ID);
  return 0;
}

static int test_mit_rejects_rtr_before_data_parse(void) {
  ProtocolMIT_Init();
  CAN_Frame command = {0};
  command.id = g_can_id;
  command.dlc = 8U;
  command.is_rtr = true;
  memset(command.data, 0xA5, sizeof(command.data));
  MotorCommand parsed = {0};
  CHECK(ProtocolMIT_Parse(&command, &parsed) == PARSE_ERR_INVALID_FRAME);
  return 0;
}
static int
test_canopen_builders_initialize_frame_metadata_and_unused_bytes(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  motor_data.Controller.torque_limit = 2.0f;
  ProtocolCANopen_Init();
  MotorStatus status = {.position = 1.5f, .velocity = -2.25f};
  CAN_Frame frame;
  memset(&frame, 0xA5, sizeof(frame));
  CHECK(ProtocolCANopen_BuildFeedback(&status, &frame));
  CHECK(frame.id == 0x180U + g_can_id);
  CHECK(frame.dlc == 8U);
  CHECK(frame.is_extended == false);
  CHECK(frame.is_rtr == false);
  CHECK(read_i32_le(&frame.data[0]) == 1500);
  CHECK(read_i32_le(&frame.data[4]) == -2250);

  memset(&frame, 0xA5, sizeof(frame));
  CHECK(ProtocolCANopen_BuildFault(0U, &frame));
  CHECK(frame.id == 0x080U + g_can_id);
  CHECK(frame.dlc == 8U);
  CHECK(frame.is_extended == false);
  CHECK(frame.is_rtr == false);

  memset(&frame, 0xA5, sizeof(frame));
  CHECK(ProtocolCANopen_BuildHeartbeat(1000U, &frame));
  CHECK(frame.id == 0x700U + g_can_id);
  CHECK(frame.dlc == 1U);
  CHECK(frame.is_extended == false);
  CHECK(frame.is_rtr == false);
  CHECK(bytes_are_zero(frame.data, 1U, 8U));

  CAN_Frame command = {0};
  command.id = 0x200U + g_can_id;
  command.dlc = 8U;
  command.data[2] = CANOPEN_MODE_POSITION;
  write_i32_le(&command.data[4], 3142);
  MotorCommand parsed = {0};
  CHECK(ProtocolCANopen_Parse(&command, &parsed) == PARSE_OK);
  CHECK(parsed.control_mode == CONTROL_MODE_POSITION);
  CHECK(parsed.position_ref > 3.141f && parsed.position_ref < 3.143f);

  command.id = 0x201U;
  CHECK(ProtocolCANopen_Parse(&command, &parsed) == PARSE_UNKNOWN_ID);
  command.id = 0x200U + g_can_id;
  command.is_extended = true;
  CHECK(ProtocolCANopen_Parse(&command, &parsed) == PARSE_UNKNOWN_ID);
  return 0;
}

static int test_canopen_sdo_downloads_use_standard_layout_and_ack(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  motor_data.Controller.torque_limit = 2.0f;
  ProtocolCANopen_Init();
  s_sent_frame_count = 0U;

  CAN_Frame frame = make_canopen_sdo(0x2BU, CANOPEN_OBJ_CONTROLWORD, 0U);
  frame.data[4] = 0x0FU;
  MotorCommand parsed = {0};
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_OK);
  CHECK(parsed.has_control_word);
  CHECK(parsed.control_word == 0x000FU);
  CHECK(s_sent_frame_count == 1U);
  CHECK(s_last_sent_frame.id == 0x580U + g_can_id);
  CHECK(s_last_sent_frame.dlc == 8U);
  CHECK(s_last_sent_frame.data[0] == 0x60U);
  CHECK(s_last_sent_frame.data[1] == 0x40U);
  CHECK(s_last_sent_frame.data[2] == 0x60U);
  CHECK(s_last_sent_frame.data[3] == 0U);
  CHECK(bytes_are_zero(s_last_sent_frame.data, 4U, 8U));

  frame = make_canopen_sdo(0x2FU, CANOPEN_OBJ_MODES_OF_OP, 0U);
  frame.data[4] = CANOPEN_MODE_CST;
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_OK);
  CHECK(s_sent_frame_count == 2U);

  frame = make_canopen_sdo(0x23U, CANOPEN_OBJ_TARGET_POSITION, 0U);
  write_i32_le(&frame.data[4], 3142);
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_OK);
  CHECK(parsed.control_mode == CONTROL_MODE_POSITION);
  CHECK(parsed.position_ref > 3.141f && parsed.position_ref < 3.143f);

  frame = make_canopen_sdo(0x2BU, CANOPEN_OBJ_TARGET_TORQUE, 0U);
  frame.data[4] = 0xF4U;
  frame.data[5] = 0x01U; /* 500 per-mille of the configured torque limit */
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_OK);
  CHECK(parsed.control_mode == CONTROL_MODE_TORQUE);
  CHECK(parsed.iq_ref > 0.999f && parsed.iq_ref < 1.001f);
  return 0;
}

static int test_canopen_sdo_rejects_bad_size_and_unknown_object(void) {
  ProtocolCANopen_Init();
  s_sent_frame_count = 0U;
  MotorCommand parsed = {0};

  CAN_Frame frame = make_canopen_sdo(0x23U, CANOPEN_OBJ_TARGET_TORQUE, 0U);
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_ERR_INVALID_FRAME);
  CHECK(s_sent_frame_count == 1U);
  CHECK(s_last_sent_frame.data[0] == 0x80U);
  CHECK(read_u32_le(&s_last_sent_frame.data[4]) == 0x06070010UL);

  frame = make_canopen_sdo(0x2FU, 0x2222U, 0U);
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_ERR_UNSUPPORTED);
  CHECK(s_sent_frame_count == 2U);
  CHECK(s_last_sent_frame.data[0] == 0x80U);
  CHECK(read_u32_le(&s_last_sent_frame.data[4]) == 0x06020000UL);

  frame = make_canopen_sdo(0x23U, CANOPEN_OBJ_TARGET_POSITION, 0U);
  frame.dlc = 7U;
  CHECK(ProtocolCANopen_Parse(&frame, &parsed) == PARSE_ERR_INVALID_FRAME);
  CHECK(s_sent_frame_count == 3U);
  CHECK(s_last_sent_frame.data[0] == 0x80U);
  CHECK(read_u32_le(&s_last_sent_frame.data[4]) == 0x06070010UL);
  return 0;
}

static int test_canopen_rpdo_torque_uses_per_mille_of_torque_limit(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  motor_data.Controller.torque_limit = 2.0f;
  ProtocolCANopen_Init();

  CAN_Frame command = {0};
  command.id = 0x200U + g_can_id;
  command.dlc = 8U;
  command.data[2] = CANOPEN_MODE_CST;
  command.data[4] = 0x0CU;
  command.data[5] = 0xFEU; /* -500 per-mille */
  MotorCommand parsed = {0};
  CHECK(ProtocolCANopen_Parse(&command, &parsed) == PARSE_OK);
  CHECK(parsed.control_mode == CONTROL_MODE_TORQUE);
  CHECK(parsed.iq_ref < -0.999f && parsed.iq_ref > -1.001f);
  return 0;
}

static int test_canopen_stopped_state_blocks_rpdo_and_sdo_commands(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  motor_data.Controller.torque_limit = 2.0f;
  ProtocolCANopen_Init();
  s_sent_frame_count = 0U;

  CAN_Frame nmt_stop = {0};
  nmt_stop.id = 0x000U;
  nmt_stop.dlc = 2U;
  nmt_stop.data[0] = 0x02U;
  nmt_stop.data[1] = g_can_id;
  MotorCommand parsed = {0};
  CHECK(ProtocolCANopen_Parse(&nmt_stop, &parsed) == PARSE_OK);

  CAN_Frame rpdo = {0};
  rpdo.id = 0x200U + g_can_id;
  rpdo.dlc = 8U;
  rpdo.data[2] = CANOPEN_MODE_CST;
  rpdo.data[4] = 0xF4U;
  rpdo.data[5] = 0x01U;
  CHECK(ProtocolCANopen_Parse(&rpdo, &parsed) == PARSE_UNKNOWN_ID);
  CHECK(s_sent_frame_count == 0U);

  CAN_Frame sdo = make_canopen_sdo(0x2BU, CANOPEN_OBJ_TARGET_TORQUE, 0U);
  sdo.data[4] = 0xF4U;
  sdo.data[5] = 0x01U;
  CHECK(ProtocolCANopen_Parse(&sdo, &parsed) == PARSE_UNKNOWN_ID);
  CHECK(s_sent_frame_count == 0U);
  return 0;
}

int main(void) {
  int failures = 0;
  failures += test_mit_builders_initialize_frame_metadata_and_unused_bytes();
  failures += test_mit_rejects_rtr_before_data_parse();
  failures +=
      test_canopen_builders_initialize_frame_metadata_and_unused_bytes();
  failures += test_canopen_sdo_downloads_use_standard_layout_and_ack();
  failures += test_canopen_sdo_rejects_bad_size_and_unknown_object();
  failures += test_canopen_rpdo_torque_uses_per_mille_of_torque_limit();
  failures += test_canopen_stopped_state_blocks_rpdo_and_sdo_commands();
  if (failures == 0) {
    printf("All cross-protocol frame builder tests PASSED\n");
  }
  return failures;
}

bool Protocol_SendFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }
  s_last_sent_frame = *frame;
  s_sent_frame_count++;
  return true;
}
