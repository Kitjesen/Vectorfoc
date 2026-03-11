#include "canopen_protocol.h"
#include "mit_protocol.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define EPSILON 1e-3f
#define ASSERT_FLOAT_NEAR(a, b, eps) assert(fabsf((a) - (b)) < (eps))

uint8_t g_can_id = 1;
uint8_t g_can_baudrate = 0;
uint8_t g_protocol_type = 0;
uint32_t g_can_timeout_ms = 0;

static int tests_passed = 0;

#define TEST(name)                                                            \
  do {                                                                        \
    printf("  Testing: %s ... ", #name);                                      \
    test_##name();                                                            \
    printf("PASS\n");                                                         \
    tests_passed++;                                                           \
  } while (0)

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

static uint16_t float_to_uint12(float x, float min_val, float max_val) {
  float span = max_val - min_val;
  float scaled = (x - min_val) / span;

  if (scaled < 0.0f) {
    scaled = 0.0f;
  }
  if (scaled > 1.0f) {
    scaled = 1.0f;
  }

  return (uint16_t)(scaled * 4095.0f);
}

static void put_i32_le(uint8_t *dst, int32_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
  dst[2] = (uint8_t)((value >> 16) & 0xFF);
  dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

static void test_mit_parse_control_frame(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  uint16_t pos_raw = float_to_uint16(1.25f, MIT_P_MIN, MIT_P_MAX);
  uint16_t vel_raw = float_to_uint12(-3.0f, MIT_V_MIN, MIT_V_MAX);
  uint16_t kp_raw = float_to_uint12(120.0f, MIT_KP_MIN, MIT_KP_MAX);
  uint16_t kd_raw = float_to_uint12(2.5f, MIT_KD_MIN, MIT_KD_MAX);
  uint16_t torque_raw = float_to_uint12(4.0f, MIT_T_MIN, MIT_T_MAX);

  ProtocolMIT_Init();

  frame.dlc = 8;
  frame.data[0] = (uint8_t)(pos_raw >> 8);
  frame.data[1] = (uint8_t)(pos_raw & 0xFF);
  frame.data[2] = (uint8_t)(vel_raw >> 4);
  frame.data[3] =
      (uint8_t)(((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
  frame.data[4] = (uint8_t)(kp_raw & 0xFF);
  frame.data[5] = (uint8_t)(kd_raw >> 4);
  frame.data[6] =
      (uint8_t)(((kd_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F));
  frame.data[7] = (uint8_t)(torque_raw & 0xFF);

  assert(ProtocolMIT_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_MIT);
  assert(cmd.has_enable_command);
  assert(cmd.enable_motor);
  ASSERT_FLOAT_NEAR(cmd.pos_setpoint, 1.25f, 0.02f);
  ASSERT_FLOAT_NEAR(cmd.vel_setpoint, -3.0f, 0.05f);
  ASSERT_FLOAT_NEAR(cmd.kp, 120.0f, 0.2f);
  ASSERT_FLOAT_NEAR(cmd.kd, 2.5f, 0.02f);
  ASSERT_FLOAT_NEAR(cmd.torque_ff, 4.0f, 0.03f);
}

static void test_mit_parse_rtr_query(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  frame.dlc = 0;
  frame.is_rtr = true;

  assert(ProtocolMIT_Parse(&frame, &cmd) == PARSE_OK);
}

static void test_mit_build_feedback(void) {
  MotorStatus status = {0};
  CAN_Frame frame = {0};
  uint16_t pos_raw = 0;
  uint16_t vel_raw = 0;
  uint16_t torque_raw = 0;

  status.position = 2.0f;
  status.velocity = -5.0f;
  status.torque = 1.5f;
  status.motor_state = 4U;
  status.fault_code = 3U;

  assert(ProtocolMIT_BuildFeedback(&status, &frame));
  pos_raw = float_to_uint16(status.position, MIT_P_MIN, MIT_P_MAX);
  vel_raw = float_to_uint12(status.velocity, MIT_V_MIN, MIT_V_MAX);
  torque_raw = float_to_uint12(status.torque, MIT_T_MIN, MIT_T_MAX);

  assert(frame.id == g_can_id);
  assert(frame.dlc == 6);
  assert(!frame.is_extended);
  assert(frame.data[0] == (uint8_t)(pos_raw >> 8));
  assert(frame.data[1] == (uint8_t)(pos_raw & 0xFF));
  assert(frame.data[2] == (uint8_t)(vel_raw >> 4));
  assert(frame.data[3] ==
         (uint8_t)(((vel_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F)));
  assert(frame.data[4] == (uint8_t)(torque_raw & 0xFF));
  assert(frame.data[5] == 0x43U);
}

static void test_mit_build_fault(void) {
  CAN_Frame frame = {0};

  assert(ProtocolMIT_BuildFault(0x123456U, &frame));
  assert(frame.id == g_can_id);
  assert(frame.dlc == 4);
  assert(!frame.is_extended);
  assert(frame.data[0] == 0xFFU);
  assert(frame.data[1] == 0x12U);
  assert(frame.data[2] == 0x34U);
  assert(frame.data[3] == 0x56U);
}

static void test_canopen_parse_position_rpdo(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = 123456;

  ProtocolCANopen_Init();

  frame.id = 0x201;
  frame.dlc = 8;
  frame.data[0] = 0x0F;
  frame.data[1] = 0x00;
  frame.data[2] = CANOPEN_MODE_POSITION;
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.has_control_word);
  assert(cmd.control_word == 0x000F);
  assert(cmd.control_mode == CONTROL_MODE_POSITION);
  assert((int32_t)cmd.position_ref == target);
}

static void test_canopen_parse_velocity_rpdo(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = -6400;

  ProtocolCANopen_Init();

  frame.id = 0x201;
  frame.dlc = 8;
  frame.data[0] = 0x0F;
  frame.data[1] = 0x00;
  frame.data[2] = CANOPEN_MODE_VELOCITY;
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_VELOCITY);
  assert((int32_t)cmd.speed_ref == target);
}

static void test_canopen_parse_torque_rpdo(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = 320;

  ProtocolCANopen_Init();

  frame.id = 0x201;
  frame.dlc = 8;
  frame.data[0] = 0x0F;
  frame.data[1] = 0x00;
  frame.data[2] = CANOPEN_MODE_CST;
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_TORQUE);
  assert((int32_t)cmd.iq_ref == target);
}

static void test_canopen_nmt_transitions_and_reset(void) {
  CAN_Frame frame = {0};
  CAN_Frame hb = {0};
  MotorCommand cmd = {0};

  ProtocolCANopen_Init();

  frame.id = 0x000;
  frame.dlc = 2;
  frame.data[1] = 0x01;

  frame.data[0] = 0x01;
  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(ProtocolCANopen_BuildHeartbeat(1000, &hb));
  assert(hb.data[0] == CANOPEN_STATE_OPERATIONAL);

  frame.data[0] = 0x02;
  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(ProtocolCANopen_BuildHeartbeat(2000, &hb));
  assert(hb.data[0] == CANOPEN_STATE_STOPPED);

  frame.data[0] = 0x80;
  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(ProtocolCANopen_BuildHeartbeat(3000, &hb));
  assert(hb.data[0] == CANOPEN_STATE_PREOPERATIONAL);

  frame.data[0] = 0x81;
  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(ProtocolCANopen_BuildHeartbeat(4000, &hb));
  assert(hb.data[0] == CANOPEN_STATE_PREOPERATIONAL);
}

static void test_canopen_parse_sdo_controlword(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  ProtocolCANopen_Init();

  frame.id = 0x601;
  frame.dlc = 8;
  frame.data[0] = (uint8_t)(CANOPEN_OBJ_CONTROLWORD & 0xFF);
  frame.data[1] = (uint8_t)(CANOPEN_OBJ_CONTROLWORD >> 8);
  frame.data[2] = 0x00;
  frame.data[4] = 0x0F;
  frame.data[5] = 0x00;

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.has_control_word);
  assert(cmd.control_word == 0x000F);
}

static void test_canopen_parse_sdo_modes_of_operation(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = 888;

  ProtocolCANopen_Init();

  frame.id = 0x601;
  frame.dlc = 8;
  frame.data[0] = (uint8_t)(CANOPEN_OBJ_MODES_OF_OP & 0xFF);
  frame.data[1] = (uint8_t)(CANOPEN_OBJ_MODES_OF_OP >> 8);
  frame.data[2] = 0x00;
  frame.data[4] = CANOPEN_MODE_VELOCITY;
  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);

  memset(&cmd, 0, sizeof(cmd));
  frame.data[0] = (uint8_t)(CANOPEN_OBJ_TARGET_VELOCITY & 0xFF);
  frame.data[1] = (uint8_t)(CANOPEN_OBJ_TARGET_VELOCITY >> 8);
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_VELOCITY);
  assert((int32_t)cmd.speed_ref == target);
}

static void test_canopen_parse_sdo_target_position(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = 4567;

  ProtocolCANopen_Init();

  frame.id = 0x601;
  frame.dlc = 8;
  frame.data[0] = (uint8_t)(CANOPEN_OBJ_TARGET_POSITION & 0xFF);
  frame.data[1] = (uint8_t)(CANOPEN_OBJ_TARGET_POSITION >> 8);
  frame.data[2] = 0x00;
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_POSITION);
  assert((int32_t)cmd.position_ref == target);
}

static void test_canopen_parse_sdo_target_velocity(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = -2200;

  ProtocolCANopen_Init();

  frame.id = 0x601;
  frame.dlc = 8;
  frame.data[0] = (uint8_t)(CANOPEN_OBJ_TARGET_VELOCITY & 0xFF);
  frame.data[1] = (uint8_t)(CANOPEN_OBJ_TARGET_VELOCITY >> 8);
  frame.data[2] = 0x00;
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_VELOCITY);
  assert((int32_t)cmd.speed_ref == target);
}

static void test_canopen_parse_sdo_target_torque(void) {
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};
  int32_t target = 64;

  ProtocolCANopen_Init();

  frame.id = 0x601;
  frame.dlc = 8;
  frame.data[0] = (uint8_t)(CANOPEN_OBJ_TARGET_TORQUE & 0xFF);
  frame.data[1] = (uint8_t)(CANOPEN_OBJ_TARGET_TORQUE >> 8);
  frame.data[2] = 0x00;
  put_i32_le(&frame.data[4], target);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.control_mode == CONTROL_MODE_TORQUE);
  assert((int32_t)cmd.iq_ref == target);
}

static void test_canopen_build_feedback(void) {
  MotorStatus status = {0};
  CAN_Frame frame = {0};

  ProtocolCANopen_Init();

  status.position = 123.0f;
  status.velocity = -45.0f;

  assert(ProtocolCANopen_BuildFeedback(&status, &frame));
  assert(frame.id == 0x181U);
  assert(frame.dlc == 8);
  assert(frame.data[0] == 123U);
  assert(frame.data[1] == 0U);
  assert(frame.data[4] == (uint8_t)0xD3U);
  assert(frame.data[5] == 0xFFU);
  assert(frame.data[6] == 0xFFU);
  assert(frame.data[7] == 0xFFU);
}

static void test_canopen_build_fault(void) {
  CAN_Frame frame = {0};

  ProtocolCANopen_Init();

  assert(ProtocolCANopen_BuildFault(0x12345678U, &frame));
  assert(frame.id == 0x081U);
  assert(frame.dlc == 8);
  assert(frame.data[0] == 0x78U);
  assert(frame.data[1] == 0x56U);
  assert(frame.data[2] == 0x34U);
  assert(frame.data[3] == 0x12U);
}

static void test_canopen_build_heartbeat(void) {
  CAN_Frame hb = {0};
  CAN_Frame frame = {0};
  MotorCommand cmd = {0};

  ProtocolCANopen_Init();

  frame.id = 0x000;
  frame.dlc = 2;
  frame.data[0] = 0x01;
  frame.data[1] = 0x01;
  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(!ProtocolCANopen_BuildHeartbeat(999, &hb));
  assert(ProtocolCANopen_BuildHeartbeat(1000, &hb));
  assert(hb.id == 0x701U);
  assert(hb.dlc == 1);
  assert(hb.data[0] == CANOPEN_STATE_OPERATIONAL);
}

int main(void) {
  printf("Running MIT/CANopen protocol tests...\n");

  TEST(mit_parse_control_frame);
  TEST(mit_parse_rtr_query);
  TEST(mit_build_feedback);
  TEST(mit_build_fault);
  TEST(canopen_parse_position_rpdo);
  TEST(canopen_parse_velocity_rpdo);
  TEST(canopen_parse_torque_rpdo);
  TEST(canopen_nmt_transitions_and_reset);
  TEST(canopen_parse_sdo_controlword);
  TEST(canopen_parse_sdo_modes_of_operation);
  TEST(canopen_parse_sdo_target_position);
  TEST(canopen_parse_sdo_target_velocity);
  TEST(canopen_parse_sdo_target_torque);
  TEST(canopen_build_feedback);
  TEST(canopen_build_fault);
  TEST(canopen_build_heartbeat);

  printf("All %d MIT/CANopen protocol tests passed.\n", tests_passed);
  return 0;
}
