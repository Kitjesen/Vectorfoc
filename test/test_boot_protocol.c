// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
#include "boot_protocol.h"
#include "bootloader.h"
#include "flash_ops.h"
#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint32_t fake_tick;
static char tx_log[256];
static uint16_t tx_len;
static uint32_t flash_write_count;
static uint32_t flash_erase_count;
static uint32_t flash_unlock_count;
static uint32_t flash_lock_count;
static uint32_t last_write_addr;
static uint32_t last_write_len;
static uint8_t last_write_data[BOOT_WRITE_BLOCK_SIZE];
static bool app_valid = true;
static uint32_t jump_count;
static bool auto_tx_complete = true;
static bool sync_tx_complete = false;
static uint8_t cdc_tx_status = 0u;

uint32_t HAL_GetTick(void) { return fake_tick; }

void HAL_Delay(uint32_t delay_ms) { fake_tick += delay_ms; }

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len) {
  assert(buf != NULL);
  if (cdc_tx_status != 0u) {
    return cdc_tx_status;
  }
  assert((uint32_t)tx_len + len < sizeof(tx_log));
  memcpy(&tx_log[tx_len], buf, len);
  tx_len = (uint16_t)(tx_len + len);
  tx_log[tx_len] = '\0';
  if (sync_tx_complete) {
    BootProto_OnTransmitComplete();
  }
  return 0u;
}

void Flash_Unlock(void) { flash_unlock_count++; }

void Flash_Lock(void) { flash_lock_count++; }

BootStatus_t Flash_EraseAppArea(void) {
  flash_erase_count++;
  return BOOT_OK;
}

BootStatus_t Flash_ErasePage(uint32_t page_num) {
  (void)page_num;
  return BOOT_OK;
}

BootStatus_t Flash_WriteData(uint32_t addr, const uint8_t *data, uint32_t len) {
  assert(data != NULL);
  flash_write_count++;
  last_write_addr = addr;
  last_write_len = len;
  memcpy(last_write_data, data, len);
  return BOOT_OK;
}

void Flash_ReadData(uint32_t addr, uint8_t *data, uint32_t len) {
  (void)addr;
  (void)data;
  (void)len;
}

uint32_t Flash_CalcCRC32(const uint8_t *data, uint32_t len) {
  (void)data;
  (void)len;
  return 0u;
}

uint32_t Flash_CRC32Update(uint32_t crc, const uint8_t *data, uint32_t len) {
  (void)data;
  (void)len;
  return crc;
}

uint32_t Flash_CalcFlashCRC32(uint32_t addr, uint32_t len) {
  (void)addr;
  (void)len;
  return 0u;
}

uint32_t Flash_CalcAppImageCRC32(uint32_t payload_len) {
  (void)payload_len;
  return 0u;
}

bool Flash_IsAddrInAppArea(uint32_t addr, uint32_t len) {
  return len > 0u && addr >= APP_ADDR_START && len - 1u <= APP_ADDR_END - addr;
}

bool Boot_CheckAppValid(void) { return app_valid; }

bool Boot_CheckUpgradeFlag(void) { return false; }
void Boot_SetUpgradeFlag(void) {}
void Boot_ClearUpgradeFlag(void) {}
bool Boot_CheckForceButton(void) { return false; }
void Boot_Main(void) {}
void Boot_EnterUpgradeMode(void) {}
void Boot_RequestUpgrade(void) {}
const AppHeader_t *Boot_GetAppHeader(void) {
  return (const AppHeader_t *)APP_HEADER_ADDR;
}

void Boot_JumpToApp(void) { jump_count++; }

static void reset_fakes(void) {
  fake_tick = 0u;
  tx_len = 0u;
  tx_log[0] = '\0';
  flash_write_count = 0u;
  flash_erase_count = 0u;
  flash_unlock_count = 0u;
  flash_lock_count = 0u;
  last_write_addr = 0u;
  last_write_len = 0u;
  memset(last_write_data, 0, sizeof(last_write_data));
  app_valid = true;
  jump_count = 0u;
  auto_tx_complete = true;
  sync_tx_complete = false;
  cdc_tx_status = 0u;
  BootProto_Init();
}

static void clear_tx(void) {
  tx_len = 0u;
  tx_log[0] = '\0';
}

static void process_data(const uint8_t *data, uint16_t len) {
  BootProto_ProcessData(data, len);
  if (auto_tx_complete) {
    BootProto_OnTransmitComplete();
  }
}

static void fragmented_write_waits_for_full_block(void) {
  static const uint8_t first_fragment[3] = {1u, 2u, 3u};
  static const uint8_t second_fragment[5] = {4u, 5u, 6u, 7u, 8u};
  static const uint8_t expected[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
  static const uint8_t command[] = "boot_write,08004000,8\n";

  reset_fakes();

  process_data(command, (uint16_t)(sizeof(command) - 1u));
  assert(strcmp(tx_log, "boot_ack,0\n") == 0);
  clear_tx();

  process_data(first_fragment, sizeof(first_fragment));
  assert(flash_write_count == 0u);
  assert(tx_len == 0u);

  process_data(second_fragment, sizeof(second_fragment));
  assert(flash_write_count == 1u);
  assert(flash_unlock_count == 1u);
  assert(flash_lock_count == 1u);
  assert(last_write_addr == APP_ADDR_START);
  assert(last_write_len == sizeof(expected));
  assert(memcmp(last_write_data, expected, sizeof(expected)) == 0);
  assert(strcmp(tx_log, "boot_ack,0\n") == 0);
}

static void write_accepts_leading_zero_data(void) {
  static const uint8_t command[] = "boot_write,08004000,8\n";
  static const uint8_t data[8] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};

  reset_fakes();
  process_data(command, (uint16_t)(sizeof(command) - 1u));
  clear_tx();

  process_data(data, sizeof(data));
  assert(flash_write_count == 1u);
  assert(last_write_len == sizeof(data));
  assert(memcmp(last_write_data, data, sizeof(data)) == 0);
  assert(strcmp(tx_log, "boot_ack,0\n") == 0);
}

static void trailing_command_ack_waits_for_tx_completion(void) {
  static const uint8_t command[] = "boot_write,08004000,8\n";
  static const uint8_t data_and_command[] = {1u,  2u,  3u,  4u,  5u,  6u,
                                             7u,  8u,  'b', 'o', 'o', 't',
                                             '_', 'i', 'n', 'f', 'o', '\n'};

  reset_fakes();
  process_data(command, (uint16_t)(sizeof(command) - 1u));
  clear_tx();

  auto_tx_complete = false;
  process_data(data_and_command, sizeof(data_and_command));
  assert(flash_write_count == 1u);
  assert(strcmp(tx_log, "boot_ack,0\n") == 0);

  BootProto_OnTransmitComplete();
  assert(strstr(tx_log, "boot_info,app_start=08004000") != NULL);
}
static void reboot_ack_requires_valid_app(void) {
  static const uint8_t command[] = "boot_reboot\n";

  reset_fakes();
  app_valid = false;
  process_data(command, (uint16_t)(sizeof(command) - 1u));
  assert(strcmp(tx_log, "boot_ack,8\n") == 0);
  assert(jump_count == 0u);
}

static void reboot_waits_for_ack_completion_and_service(void) {
  static const uint8_t command[] = "boot_reboot\n";

  reset_fakes();
  auto_tx_complete = false;
  process_data(command, (uint16_t)(sizeof(command) - 1u));
  assert(strcmp(tx_log, "boot_ack,0,rebooting\n") == 0);
  assert(jump_count == 0u);

  BootProto_Service();
  assert(jump_count == 0u);

  BootProto_OnTransmitComplete();
  assert(jump_count == 0u);

  BootProto_Service();
  assert(jump_count == 1u);
}

static void sync_tx_completion_does_not_leave_queue_busy(void) {
  reset_fakes();
  sync_tx_complete = true;

  BootProto_SendResponse("first");
  BootProto_SendResponse("second");

  assert(strcmp(tx_log, "first\nsecond\n") == 0);
  assert(jump_count == 0u);
}

static void queued_async_messages_flush_in_order(void) {
  reset_fakes();
  auto_tx_complete = false;

  BootProto_SendResponse("one");
  BootProto_SendResponse("two");
  BootProto_SendResponse("three");

  assert(strcmp(tx_log, "one\n") == 0);
  BootProto_OnTransmitComplete();
  assert(strcmp(tx_log, "one\ntwo\n") == 0);
  BootProto_OnTransmitComplete();
  assert(strcmp(tx_log, "one\ntwo\nthree\n") == 0);
}

static void full_queue_reboot_does_not_arm_jump(void) {
  static const uint8_t command[] = "boot_reboot\n";

  reset_fakes();
  auto_tx_complete = false;
  cdc_tx_status = 1u;

  BootProto_SendResponse("q0");
  BootProto_SendResponse("q1");
  BootProto_SendResponse("q2");
  BootProto_SendResponse("q3");
  process_data(command, (uint16_t)(sizeof(command) - 1u));

  cdc_tx_status = 0u;
  for (uint8_t i = 0u; i < 8u; ++i) {
    BootProto_Service();
    BootProto_OnTransmitComplete();
  }
  assert(strstr(tx_log, "rebooting") == NULL);
  assert(jump_count == 0u);
}

static void queued_usb_data_is_processed_only_by_service(void) {
  static const uint8_t command[] = "boot_erase\n";

  reset_fakes();
  assert(BootProto_QueueData(command, (uint16_t)(sizeof(command) - 1u)));
  assert(flash_erase_count == 0u);
  assert(flash_unlock_count == 0u);
  assert(tx_len == 0u);

  BootProto_Service();
  assert(flash_erase_count == 1u);
  assert(flash_unlock_count == 1u);
  assert(flash_lock_count == 1u);
  assert(strcmp(tx_log, "boot_ack,0\n") == 0);
}

static void rx_queue_overflow_aborts_the_partial_transaction(void) {
  static const uint8_t packet[] = "boot_info\n";

  reset_fakes();
  for (uint8_t i = 0u; i < 8u; ++i) {
    assert(BootProto_QueueData(packet, (uint16_t)(sizeof(packet) - 1u)));
  }
  assert(!BootProto_QueueData(packet, (uint16_t)(sizeof(packet) - 1u)));
  assert(BootProto_GetReceiveOverflowCount() == 1u);

  BootProto_Service();
  assert(strcmp(tx_log, "boot_ack,9\n") == 0);
}

static void oversized_rx_packet_is_rejected_without_counting_as_overflow(void) {
  uint8_t packet[65] = {0};

  reset_fakes();
  assert(!BootProto_QueueData(packet, sizeof(packet)));
  assert(BootProto_GetReceiveOverflowCount() == 0u);
}

static void overlong_command_is_rejected_and_parser_recovers(void) {
  uint8_t overlong[BOOT_RX_BUFFER_SIZE + 1u];
  static const uint8_t next_command[] = "boot_info\n";

  reset_fakes();
  memset(overlong, 'x', sizeof(overlong));
  overlong[sizeof(overlong) - 1u] = '\n';
  process_data(overlong, sizeof(overlong));
  assert(strcmp(tx_log, "boot_ack,7\n") == 0);

  clear_tx();
  process_data(next_command, (uint16_t)(sizeof(next_command) - 1u));
  assert(strstr(tx_log, "boot_info,app_start=08004000") != NULL);
}

static void timeout_emits_one_canonical_ack(void) {
  static const uint8_t command[] = "boot_write,08004000,8\n";

  reset_fakes();
  process_data(command, (uint16_t)(sizeof(command) - 1u));
  clear_tx();

  fake_tick = BOOT_PROTOCOL_TIMEOUT_MS + 1u;
  assert(BootProto_CheckTimeout(fake_tick));
  assert(strcmp(tx_log, "boot_ack,6\n") == 0);
}

int main(void) {
  queued_usb_data_is_processed_only_by_service();
  rx_queue_overflow_aborts_the_partial_transaction();
  oversized_rx_packet_is_rejected_without_counting_as_overflow();
  overlong_command_is_rejected_and_parser_recovers();
  timeout_emits_one_canonical_ack();
  fragmented_write_waits_for_full_block();
  write_accepts_leading_zero_data();
  trailing_command_ack_waits_for_tx_completion();
  reboot_ack_requires_valid_app();
  reboot_waits_for_ack_completion_and_service();
  sync_tx_completion_does_not_leave_queue_busy();
  queued_async_messages_flush_in_order();
  full_queue_reboot_does_not_arm_jump();
  return 0;
}
