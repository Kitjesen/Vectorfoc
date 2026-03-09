#include "inovxio_protocol.h"
#include "vofa_bus_scan.h"
#include <assert.h>
#include <stdio.h>

static int tests_passed = 0;

#define TEST(name)                                                            \
  do {                                                                        \
    printf("  Testing: %s ... ", #name);                                      \
    test_##name();                                                            \
    printf("PASS\n");                                                         \
    tests_passed++;                                                           \
  } while (0)

static void store_u32_le(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void test_bus_scan_reports_local_then_remote_then_done(void) {
  VofaBusScanState state = {0};
  VofaBusScanEvent event = {0};
  CAN_Frame frame = {0};

  VofaBusScan_Begin(&state, 100U, 50U, 5U, 0x11111111U, 0x22222222U);

  assert(VofaBusScan_PollEvent(&state, 100U, &event));
  assert(event.type == VOFA_BUS_SCAN_EVENT_NODE);
  assert(event.node.node_id == 5U);

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | (7U << 8) |
             VOFA_BUS_SCAN_TARGET_HOST;
  frame.is_extended = true;
  frame.dlc = 8;
  store_u32_le(&frame.data[0], 0x12345678U);
  store_u32_le(&frame.data[4], 0x9ABCDEF0U);

  VofaBusScan_ObserveFrame(&state, &frame);

  assert(VofaBusScan_PollEvent(&state, 120U, &event));
  assert(event.type == VOFA_BUS_SCAN_EVENT_NODE);
  assert(event.node.node_id == 7U);
  assert(event.node.uid_word0 == 0x12345678U);
  assert(event.node.uid_word1 == 0x9ABCDEF0U);

  assert(!VofaBusScan_PollEvent(&state, 149U, &event));
  assert(VofaBusScan_PollEvent(&state, 150U, &event));
  assert(event.type == VOFA_BUS_SCAN_EVENT_DONE);
  assert(event.total_nodes == 2U);
}

static void test_bus_scan_ignores_duplicate_and_non_discovery_frames(void) {
  VofaBusScanState state = {0};
  VofaBusScanEvent event = {0};
  CAN_Frame frame = {0};

  VofaBusScan_Begin(&state, 0U, 20U, 3U, 0xAAAA5555U, 0xBBBB6666U);
  assert(VofaBusScan_PollEvent(&state, 0U, &event));

  frame.id = ((uint32_t)PRIVATE_CMD_MOTOR_CTRL << 24) | (9U << 8) |
             VOFA_BUS_SCAN_TARGET_HOST;
  frame.is_extended = true;
  frame.dlc = 8;
  VofaBusScan_ObserveFrame(&state, &frame);

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | (3U << 8) |
             VOFA_BUS_SCAN_TARGET_HOST;
  VofaBusScan_ObserveFrame(&state, &frame);

  assert(!VofaBusScan_PollEvent(&state, 10U, &event));
  assert(VofaBusScan_PollEvent(&state, 20U, &event));
  assert(event.type == VOFA_BUS_SCAN_EVENT_DONE);
  assert(event.total_nodes == 1U);
}

int main(void) {
  printf("Running VOFA bus scan tests...\n");

  TEST(bus_scan_reports_local_then_remote_then_done);
  TEST(bus_scan_ignores_duplicate_and_non_discovery_frames);

  printf("All %d VOFA bus scan tests passed.\n", tests_passed);
  return 0;
}
