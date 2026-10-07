#include "boot_protocol.h"
#include "bootloader.h"
#include "flash_ops.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint32_t fake_tick;
static char tx_log[256];
static uint16_t tx_len;
static unsigned flash_write_count;
static unsigned flash_erase_count;
static uint32_t last_write_addr;
static uint32_t last_write_len;
static uint8_t last_write_data[BOOT_WRITE_BLOCK_SIZE];
static bool app_valid;
static unsigned jump_count;
static bool auto_tx_complete;

uint32_t HAL_GetTick(void) { return fake_tick; }
void HAL_Delay(uint32_t delay_ms) { fake_tick += delay_ms; }

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len) {
  assert((uint32_t)tx_len + len < sizeof(tx_log));
  memcpy(tx_log + tx_len, buf, len);
  tx_len = (uint16_t)(tx_len + len);
  tx_log[tx_len] = '\0';
  return 0u;
}

void Flash_Unlock(void) {}
void Flash_Lock(void) {}
BootStatus_t Flash_EraseAppArea(void) {
  ++flash_erase_count;
  return BOOT_OK;
}
BootStatus_t Flash_WriteData(uint32_t addr, const uint8_t *data, uint32_t len) {
  ++flash_write_count;
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
uint32_t Flash_CalcAppImageCRC32(uint32_t len) {
  (void)len;
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
void Boot_JumpToApp(void) { ++jump_count; }

static void reset_protocol(void) {
  fake_tick = 0u;
  tx_len = 0u;
  tx_log[0] = '\0';
  flash_write_count = 0u;
  flash_erase_count = 0u;
  jump_count = 0u;
  app_valid = true;
  auto_tx_complete = true;
  BootProto_Init();
}

static void process(const uint8_t *data, uint16_t len) {
  BootProto_ProcessData(data, len);
  if (auto_tx_complete)
    BootProto_OnTransmitComplete();
}

static void fragmented_write_waits_for_full_block(void) {
  static const uint8_t command[] = "boot_write,08004000,8\n";
  static const uint8_t first[] = {1u, 2u, 3u};
  static const uint8_t second[] = {4u, 5u, 6u, 7u, 8u};
  static const uint8_t expected[] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};

  reset_protocol();
  process(command, sizeof(command) - 1u);
  tx_len = 0u;
  tx_log[0] = '\0';
  process(first, sizeof(first));
  assert(flash_write_count == 0u);
  process(second, sizeof(second));
  assert(flash_write_count == 1u);
  assert(last_write_addr == APP_ADDR_START);
  assert(last_write_len == sizeof(expected));
  assert(memcmp(last_write_data, expected, sizeof(expected)) == 0);
}

static void queued_usb_data_runs_only_in_service(void) {
  static const uint8_t command[] = "boot_erase\n";
  reset_protocol();
  assert(BootProto_QueueData(command, sizeof(command) - 1u));
  assert(flash_erase_count == 0u);
  BootProto_Service();
  assert(flash_erase_count == 1u);
}

static void receive_queue_overflow_aborts_transaction(void) {
  static const uint8_t command[] = "boot_info\n";
  reset_protocol();
  for (unsigned i = 0; i < 8u; ++i)
    assert(BootProto_QueueData(command, sizeof(command) - 1u));
  assert(!BootProto_QueueData(command, sizeof(command) - 1u));
  assert(BootProto_GetReceiveOverflowCount() == 1u);
  BootProto_Service();
  assert(strcmp(tx_log, "boot_ack,9\n") == 0);
}

static void timeout_emits_one_canonical_ack(void) {
  static const uint8_t command[] = "boot_write,08004000,8\n";
  reset_protocol();
  process(command, sizeof(command) - 1u);
  tx_len = 0u;
  tx_log[0] = '\0';
  assert(BootProto_CheckTimeout(BOOT_PROTOCOL_TIMEOUT_MS + 1u));
  assert(strcmp(tx_log, "boot_ack,6\n") == 0);
}

static void reboot_waits_for_ack_completion_and_service(void) {
  static const uint8_t command[] = "boot_reboot\n";
  reset_protocol();
  auto_tx_complete = false;
  process(command, sizeof(command) - 1u);
  assert(jump_count == 0u);
  BootProto_Service();
  assert(jump_count == 0u);
  BootProto_OnTransmitComplete();
  assert(jump_count == 0u);
  BootProto_Service();
  assert(jump_count == 1u);
}

int main(void) {
  fragmented_write_waits_for_full_block();
  queued_usb_data_runs_only_in_service();
  receive_queue_overflow_aborts_transaction();
  timeout_emits_one_canonical_ack();
  reboot_waits_for_ack_completion_and_service();
  return 0;
}
