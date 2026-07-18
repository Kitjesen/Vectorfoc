// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file boot_protocol.c
 * @brief OTA 升级协议实现
 */
#include "boot_protocol.h"
#include "bootloader.h"
#include "flash_ops.h"
#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * 私有变量
 * ============================================================================
 */
static ProtoContext_t s_ctx;
static uint8_t s_tx_buf[96];
#define BOOT_TX_QUEUE_LEN 4u
#define BOOT_TX_MSG_MAX sizeof(s_tx_buf)
#define BOOT_RX_QUEUE_LEN 8u
#define BOOT_RX_PACKET_MAX 64u

typedef struct {
  uint8_t data[BOOT_TX_MSG_MAX];
  uint16_t len;
  bool queued;
  bool in_flight;
} BootTxSlot_t;

typedef struct {
  uint8_t data[BOOT_RX_PACKET_MAX];
  uint16_t len;
} BootRxSlot_t;

static BootTxSlot_t s_tx_queue[BOOT_TX_QUEUE_LEN];
static uint8_t s_tx_head;
static uint8_t s_tx_tail;
static bool s_tx_busy;
static bool s_reboot_pending;

static BootRxSlot_t s_rx_queue[BOOT_RX_QUEUE_LEN];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_rx_count;
static volatile uint32_t s_rx_overflow_count;
static volatile bool s_rx_overflow_pending;
static bool s_rx_line_overflow;

static void flush_tx_queue(void);
static bool dequeue_rx(uint8_t *data, uint16_t *len);

static uint32_t boot_critical_enter(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void boot_critical_exit(uint32_t primask) { __set_PRIMASK(primask); }

static bool append_char(uint16_t *pos, char value) {
  if (*pos >= sizeof(s_tx_buf) - 1u) {
    return false;
  }
  s_tx_buf[(*pos)++] = (uint8_t)value;
  return true;
}

static bool tx_status_ok(uint8_t status) { return status == 0u; }

static void flush_tx_queue(void) {
  while (true) {
    uint32_t primask = boot_critical_enter();
    if (s_tx_busy) {
      boot_critical_exit(primask);
      return;
    }

    BootTxSlot_t *slot = &s_tx_queue[s_tx_tail];
    if (!slot->queued || slot->len == 0u) {
      boot_critical_exit(primask);
      return;
    }

    slot->in_flight = true;
    s_tx_busy = true;
    boot_critical_exit(primask);

    uint8_t status = CDC_Transmit_FS(slot->data, slot->len);
    if (!tx_status_ok(status)) {
      primask = boot_critical_enter();
      slot->in_flight = false;
      s_tx_busy = false;
      boot_critical_exit(primask);
      return;
    }
  }
}

static bool enqueue_tx(const uint8_t *data, uint16_t len) {
  if (data == NULL || len == 0u || len > BOOT_TX_MSG_MAX) {
    return false;
  }

  uint32_t primask = boot_critical_enter();
  BootTxSlot_t *slot = &s_tx_queue[s_tx_head];
  if (slot->queued) {
    boot_critical_exit(primask);
    return false;
  }
  memcpy(slot->data, data, len);
  slot->len = len;
  slot->queued = true;
  slot->in_flight = false;
  s_tx_head = (uint8_t)((s_tx_head + 1u) % BOOT_TX_QUEUE_LEN);
  boot_critical_exit(primask);

  flush_tx_queue();
  return true;
}

void BootProto_OnTransmitComplete(void) {
  uint32_t primask = boot_critical_enter();
  BootTxSlot_t *slot = &s_tx_queue[s_tx_tail];
  if (slot->queued && slot->in_flight) {
    memset(slot, 0, sizeof(*slot));
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) % BOOT_TX_QUEUE_LEN);
  }
  s_tx_busy = false;
  boot_critical_exit(primask);
  flush_tx_queue();
}

static bool append_text(uint16_t *pos, const char *text) {
  while (*text != '\0') {
    if (!append_char(pos, *text++)) {
      return false;
    }
  }
  return true;
}

static bool append_u32_dec(uint16_t *pos, uint32_t value) {
  char digits[10];
  uint8_t count = 0;
  do {
    digits[count++] = (char)('0' + (value % 10u));
    value /= 10u;
  } while (value != 0u);

  while (count > 0u) {
    if (!append_char(pos, digits[--count])) {
      return false;
    }
  }
  return true;
}

static bool append_u32_hex(uint16_t *pos, uint32_t value) {
  static const char hex[] = "0123456789ABCDEF";
  for (int shift = 28; shift >= 0; shift -= 4) {
    if (!append_char(pos, hex[(value >> shift) & 0xFu])) {
      return false;
    }
  }
  return true;
}

static bool send_buffer(uint16_t len) {
  if (append_char(&len, '\n')) {
    return enqueue_tx(s_tx_buf, len);
  }
  return false;
}

static bool parse_digit(char c, uint32_t base, uint32_t *digit) {
  if (c >= '0' && c <= '9') {
    *digit = (uint32_t)(c - '0');
  } else if (base == 16u && c >= 'a' && c <= 'f') {
    *digit = (uint32_t)(c - 'a') + 10u;
  } else if (base == 16u && c >= 'A' && c <= 'F') {
    *digit = (uint32_t)(c - 'A') + 10u;
  } else {
    return false;
  }
  return *digit < base;
}

static bool parse_u32(const char **cursor, uint32_t base, uint32_t *value) {
  const char *p = *cursor;
  uint32_t result = 0;
  uint32_t digit;
  bool has_digit = false;

  if (base == 16u && p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
    p += 2;
  }

  while (parse_digit(*p, base, &digit)) {
    if (result > (UINT32_MAX - digit) / base) {
      return false;
    }
    result = result * base + digit;
    has_digit = true;
    ++p;
  }

  if (!has_digit) {
    return false;
  }
  *cursor = p;
  *value = result;
  return true;
}

/* ============================================================================
 * 发送函数
 * ============================================================================
 */
static bool send_response_checked(const char *msg) {
  if (msg == NULL)
    return false;
  uint16_t len = 0;
  if (append_text(&len, msg)) {
    return send_buffer(len);
  }
  return false;
}

void BootProto_SendResponse(const char *msg) {
  (void)send_response_checked(msg);
}

static bool send_ack_checked(BootStatus_t status) {
  uint16_t len = 0;
  if (append_text(&len, "boot_ack,") &&
      append_u32_dec(&len, (uint32_t)status)) {
    return send_buffer(len);
  }
  return false;
}

void BootProto_SendAck(BootStatus_t status) { (void)send_ack_checked(status); }

void BootProto_SendReady(void) { (void)send_response_checked("boot_ready"); }

/* ============================================================================
 * 初始化与重置
 * ============================================================================
 */
void BootProto_Init(void) {
  memset(&s_ctx, 0, sizeof(s_ctx));
  memset(s_tx_queue, 0, sizeof(s_tx_queue));
  s_tx_head = 0u;
  s_tx_tail = 0u;
  s_tx_busy = false;
  s_reboot_pending = false;
  memset(s_rx_queue, 0, sizeof(s_rx_queue));
  s_rx_head = 0u;
  s_rx_tail = 0u;
  s_rx_count = 0u;
  s_rx_overflow_count = 0u;
  s_rx_overflow_pending = false;
  s_rx_line_overflow = false;
  s_ctx.state = PROTO_STATE_IDLE;
}

void BootProto_Reset(void) {
  s_ctx.state = PROTO_STATE_IDLE;
  s_ctx.rx_pos = 0;
  s_ctx.write_addr = 0;
  s_ctx.write_len = 0;
  s_ctx.received_len = 0;
  s_rx_line_overflow = false;
}

/* ============================================================================
 * 命令解析
 * ============================================================================
 */
static void handle_command(const char *cmd) {
  s_ctx.last_activity = HAL_GetTick();

  /* boot_erase - 擦除 App 区域 */
  if (strcmp(cmd, "boot_erase") == 0) {
    Flash_Unlock();
    BootStatus_t status = Flash_EraseAppArea();
    Flash_Lock();
    BootProto_SendAck(status);
    return;
  }

  /* boot_write,addr,len - 准备接收数据 */
  if (strncmp(cmd, "boot_write,", 11) == 0) {
    uint32_t addr = 0, len = 0;
    const char *cursor = cmd + 11;
    if (parse_u32(&cursor, 16u, &addr) && *cursor++ == ',' &&
        parse_u32(&cursor, 10u, &len) && *cursor == '\0') {
      if (len == 0u || len > BOOT_WRITE_BLOCK_SIZE || (len & 0x7u) != 0u ||
          (addr & 0x7u) != 0u || !Flash_IsAddrInAppArea(addr, len)) {
        BootProto_SendAck(BOOT_ERR_INVALID_ADDR);
        BootProto_Reset();
        return;
      }

      s_ctx.write_addr = addr;
      s_ctx.write_len = len;
      s_ctx.received_len = 0;
      s_ctx.state = PROTO_STATE_WAIT_DATA;

      /* 发送 ACK 表示准备好接收数据 */
      BootProto_SendAck(BOOT_OK);
    } else {
      BootProto_SendAck(BOOT_ERR_INVALID_CMD);
    }
    return;
  }

  /* boot_verify,crc - 校验 CRC */
  if (strncmp(cmd, "boot_verify,", 12) == 0) {
    uint32_t expected_crc = 0;
    uint32_t size = 0;
    const char *cursor = cmd + 12;

    if (parse_u32(&cursor, 16u, &expected_crc) &&
        (*cursor == '\0' ||
         (*cursor++ == ',' && parse_u32(&cursor, 10u, &size) &&
          *cursor == '\0'))) {
      /* 如果没有提供 size，使用 App Header 中的 size */
      const AppHeader_t *header = (const AppHeader_t *)APP_HEADER_ADDR;
      uint32_t header_payload_start = APP_HEADER_ADDR + sizeof(AppHeader_t);
      uint32_t max_payload_size = APP_ADDR_END + 1u - header_payload_start;

      if (header->magic != APP_MAGIC_NUMBER || header->size == 0u ||
          header->size > max_payload_size || header->reserved[0] != 0u ||
          header->reserved[1] != 0u || header->reserved[2] != 0u) {
        BootProto_SendAck(BOOT_ERR_APP_INVALID);
        return;
      }
      if (size == 0u) {
        size = header->size;
      }
      if (size != header->size || expected_crc != header->crc32) {
        BootProto_SendAck(BOOT_ERR_INVALID_CMD);
        return;
      }

      uint32_t payload_start =
          APP_ADDR_START + APP_HEADER_OFFSET + sizeof(AppHeader_t);
      if (size > 0u && Flash_IsAddrInAppArea(payload_start, size)) {
        uint32_t calc_crc = Flash_CalcAppImageCRC32(size);

        if (calc_crc == expected_crc) {
          BootProto_SendAck(BOOT_OK);
        } else {
          uint16_t len = 0;
          if (append_text(&len, "boot_ack,") &&
              append_u32_dec(&len, BOOT_ERR_CRC_MISMATCH) &&
              append_text(&len, ",calc=") && append_u32_hex(&len, calc_crc) &&
              append_text(&len, ",exp=") &&
              append_u32_hex(&len, expected_crc)) {
            (void)send_buffer(len);
          }
        }
      } else {
        BootProto_SendAck(BOOT_ERR_INVALID_CMD);
      }
    } else {
      BootProto_SendAck(BOOT_ERR_INVALID_CMD);
    }
    return;
  }

  /* boot_reboot - 重启到 App */
  if (strcmp(cmd, "boot_reboot") == 0) {
    if (!Boot_CheckAppValid()) {
      BootProto_SendAck(BOOT_ERR_APP_INVALID);
      return;
    }
    if (send_response_checked("boot_ack,0,rebooting")) {
      uint32_t primask = boot_critical_enter();
      s_reboot_pending = true;
      boot_critical_exit(primask);
    }
    return;
  }

  /* boot_info - 获取 Bootloader 信息 */
  if (strcmp(cmd, "boot_info") == 0) {
    uint16_t len = 0;
    if (append_text(&len, "boot_info,app_start=") &&
        append_u32_hex(&len, APP_ADDR_START) &&
        append_text(&len, ",app_size=") && append_u32_dec(&len, APP_SIZE)) {
      (void)send_buffer(len);
    }
    return;
  }

  /* 未知命令 */
  BootProto_SendAck(BOOT_ERR_INVALID_CMD);
}

/* ============================================================================
 * 数据处理
 * ============================================================================
 */
bool BootProto_QueueData(const uint8_t *data, uint16_t len) {
  if (data == NULL || len == 0u || len > BOOT_RX_PACKET_MAX) {
    return false;
  }

  bool queued = false;
  uint32_t primask = boot_critical_enter();
  if (s_rx_count < BOOT_RX_QUEUE_LEN) {
    BootRxSlot_t *slot = &s_rx_queue[s_rx_head];
    memcpy(slot->data, data, len);
    slot->len = len;
    s_rx_head = (uint8_t)((s_rx_head + 1u) % BOOT_RX_QUEUE_LEN);
    s_rx_count++;
    queued = true;
  } else {
    s_rx_overflow_count++;
    s_rx_overflow_pending = true;
  }
  boot_critical_exit(primask);
  return queued;
}

uint32_t BootProto_GetReceiveOverflowCount(void) {
  uint32_t count;
  uint32_t primask = boot_critical_enter();
  count = s_rx_overflow_count;
  boot_critical_exit(primask);
  return count;
}

static bool dequeue_rx(uint8_t *data, uint16_t *len) {
  if (data == NULL || len == NULL) {
    return false;
  }

  bool dequeued = false;
  uint32_t primask = boot_critical_enter();
  if (s_rx_count > 0u) {
    BootRxSlot_t *slot = &s_rx_queue[s_rx_tail];
    *len = slot->len;
    memcpy(data, slot->data, *len);
    slot->len = 0u;
    s_rx_tail = (uint8_t)((s_rx_tail + 1u) % BOOT_RX_QUEUE_LEN);
    s_rx_count--;
    dequeued = true;
  }
  boot_critical_exit(primask);
  return dequeued;
}

void BootProto_ProcessData(const uint8_t *data, uint16_t len) {
  flush_tx_queue();
  s_ctx.last_activity = HAL_GetTick();

  if (data == NULL || len == 0u) {
    BootProto_SendAck(BOOT_ERR_INVALID_CMD);
    BootProto_Reset();
    return;
  }

  if (s_ctx.state == PROTO_STATE_WAIT_DATA) {
    /* 接收二进制数据 */
    uint32_t remaining = s_ctx.write_len - s_ctx.received_len;
    uint32_t to_copy = (len < remaining) ? len : remaining;

    if (to_copy > 0u) {
      memcpy(&s_ctx.write_buf[s_ctx.received_len], data, to_copy);
      s_ctx.received_len += to_copy;
    }

    /* 检查是否接收完成 */
    if (s_ctx.received_len >= s_ctx.write_len) {
      Flash_Unlock();
      BootStatus_t status =
          Flash_WriteData(s_ctx.write_addr, s_ctx.write_buf, s_ctx.write_len);
      Flash_Lock();

      BootProto_SendAck(status);
      BootProto_Reset();

      if (status == BOOT_OK && len > to_copy) {
        BootProto_ProcessData(&data[to_copy], (uint16_t)(len - to_copy));
      }
    }

    return;
  }

  /* 文本命令模式 - 按行解析 */
  for (uint16_t i = 0; i < len; i++) {
    uint8_t c = data[i];

    if (c == '\n' || c == '\r') {
      if (s_rx_line_overflow) {
        BootProto_SendAck(BOOT_ERR_INVALID_CMD);
        BootProto_Reset();
        continue;
      }
      if (s_ctx.rx_pos > 0) {
        s_ctx.rx_buf[s_ctx.rx_pos] = '\0';
        handle_command((const char *)s_ctx.rx_buf);
        s_ctx.rx_pos = 0;
      }
    } else {
      if (s_ctx.rx_pos < sizeof(s_ctx.rx_buf) - 1 && !s_rx_line_overflow) {
        s_ctx.rx_buf[s_ctx.rx_pos++] = c;
      } else {
        /* Do not parse a truncated command as if it were valid. */
        s_rx_line_overflow = true;
      }
    }
  }
}

void BootProto_Service(void) {
  uint8_t data[BOOT_RX_PACKET_MAX];
  uint16_t len = 0u;

  uint32_t primask = boot_critical_enter();
  bool rx_overflowed = s_rx_overflow_pending;
  if (rx_overflowed) {
    /* The packet sequence is no longer trustworthy.  Drop retained packets
     * and reset any partial write transaction before reporting the error. */
    s_rx_overflow_pending = false;
    s_rx_head = s_rx_tail;
    s_rx_count = 0u;
  }
  boot_critical_exit(primask);

  if (rx_overflowed) {
    BootProto_Reset();
    BootProto_SendAck(BOOT_ERR_RX_OVERFLOW);
    return;
  }

  while (dequeue_rx(data, &len)) {
    BootProto_ProcessData(data, len);
  }

  flush_tx_queue();

  if (!s_reboot_pending) {
    return;
  }

  primask = boot_critical_enter();
  const BootTxSlot_t *slot = &s_tx_queue[s_tx_tail];
  bool can_jump = !s_tx_busy && !slot->queued;
  if (can_jump) {
    s_reboot_pending = false;
  }
  boot_critical_exit(primask);

  if (can_jump) {
    Boot_JumpToApp();
  }
}

/* ============================================================================
 * 超时检查
 * ============================================================================
 */
bool BootProto_CheckTimeout(uint32_t current_tick) {
  flush_tx_queue();
  if (s_ctx.state == PROTO_STATE_WAIT_DATA) {
    if (current_tick - s_ctx.last_activity > BOOT_PROTOCOL_TIMEOUT_MS) {
      BootProto_SendAck(BOOT_ERR_TIMEOUT);
      BootProto_Reset();
      return true;
    }
  }
  return false;
}
