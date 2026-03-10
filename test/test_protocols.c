#include "canopen_protocol.h"
#include "mit_protocol.h"
#include "motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

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
  frame.data[4] = (uint8_t)(target & 0xFF);
  frame.data[5] = (uint8_t)((target >> 8) & 0xFF);
  frame.data[6] = (uint8_t)((target >> 16) & 0xFF);
  frame.data[7] = (uint8_t)((target >> 24) & 0xFF);

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(cmd.has_control_word);
  assert(cmd.control_word == 0x000F);
  assert(cmd.control_mode == CONTROL_MODE_POSITION);
  assert((int32_t)cmd.position_ref == target);
}

static void test_canopen_nmt_start_updates_heartbeat(void) {
  CAN_Frame frame = {0};
  CAN_Frame hb = {0};
  MotorCommand cmd = {0};

  ProtocolCANopen_Init();

  frame.id = 0x000;
  frame.dlc = 2;
  frame.data[0] = 0x01;
  frame.data[1] = 0x01;

  assert(ProtocolCANopen_Parse(&frame, &cmd) == PARSE_OK);
  assert(!ProtocolCANopen_BuildHeartbeat(999, &hb));
  assert(ProtocolCANopen_BuildHeartbeat(1000, &hb));
  assert(hb.id == 0x701);
  assert(hb.dlc == 1);
  assert(hb.data[0] == CANOPEN_STATE_OPERATIONAL);
}

int main(void) {
  printf("Running protocol parser tests...\n");

  TEST(mit_parse_control_frame);
  TEST(mit_parse_rtr_query);
  TEST(canopen_parse_position_rpdo);
  TEST(canopen_nmt_start_updates_heartbeat);

  printf("All %d protocol tests passed.\n", tests_passed);
  return 0;
}
