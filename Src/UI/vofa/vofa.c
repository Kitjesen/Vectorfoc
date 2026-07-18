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
 * @file    vofa.c
 * @brief   VectorStudio
 *
 * @details  VectorStudio :
 *
 *   1.  (FireWater , 1kHz)
 *      - ISR sample → Scope  → Task
 *      - 12  float + 4
 *
 *   2. state ( → PC, driver + 10Hz )
 *      - fw_version=X.Y.Z     ( / )
 *      - calib_step=N          (calibration)
 *      - calib_done=1          (calibrationdone)
 *      - calib_error=MSG       (calibration)
 *      - fault=CODE,SEVERITY   (fault)
 *      - fault_clear=all       (fault)
 *      - param=INDEX,VALUE     (param)
 *      - ack=studio,VERSION    ()
 *
 *   3.  (PC → )
 *      - motor: motor_enable, set_ctrl_mode, calib, clear_fault
 *      - :   set_Iq, set_vel, set_pos, set_torque
 *      - PID : set_vel_kp, set_vel_ki, set_pos_kp, set_current_ctrl_bw
 *      - LADRC:    set_ladrc_en, set_ladrc_wo, set_ladrc_wc, set_ladrc_b0
 *      - : set_cogging_calib, set_cogging_enable
 *      - :     save_flash, set_zero, get_version, handshake, get_param
 */
#include "vofa.h"
#include "bootloader.h"
#include "config.h"
#include "control/cogging.h"
#include "control/control.h"
#include "executor.h"
#include "control/ladrc.h"
#include "fault_def.h"
#include "hal_encoder.h"
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "platform.h"
#include "safety_control.h"
#include "usbd_cdc_if.h"
#include "version.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* ============================================================================
 *
 * ============================================================================
 */
#define MAX_TXBUFFER_SIZE 64
#define MAX_RXBUFFER_SIZE 256
#define TEXT_LINE_MAX 128
#define VOFA_TX_QUEUE_DEPTH 5
#define VOFA_TX_SLOT_SIZE (TEXT_LINE_MAX + 2)
#define VOFA_RX_QUEUE_DEPTH 4
#define VOFA_RX_SLOT_SIZE 64

typedef struct {
  uint8_t data[VOFA_TX_SLOT_SIZE];
  uint16_t len;
  bool queued;
  bool in_flight;
} VofaTxSlot;

typedef struct {
  uint8_t data[VOFA_RX_SLOT_SIZE];
  uint16_t len;
} VofaRxSlot;

static uint8_t send_buf[MAX_TXBUFFER_SIZE];
static uint8_t receive_buf[MAX_RXBUFFER_SIZE];
static uint16_t cnt = 0;
static VofaTxSlot s_tx_queue[VOFA_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile uint8_t s_tx_count;
static volatile bool s_tx_busy;
static volatile bool s_boot_upgrade_pending;
static volatile bool s_save_flash_result_pending;
static volatile bool s_save_flash_failure_reported;
static volatile bool s_param_sync_active;
static volatile uint32_t s_param_sync_index;

static VofaRxSlot s_rx_queue[VOFA_RX_QUEUE_DEPTH];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint8_t s_rx_count;
static volatile uint32_t s_rx_overflow_count;
static uint32_t s_rx_overflow_reported_count;

static void Studio_ProcessParamSync(void);
/* ============================================================================
 *
 * ============================================================================
 */
#define USART_OR_CDC 1

static void Vofa_TryStartTransmit(void) {
#if USART_OR_CDC == 0
  return;
#elif USART_OR_CDC == 1
  VofaTxSlot *slot = NULL;

  CRITICAL_SECTION_BEGIN();
  if (!s_tx_busy && s_tx_count > 0U) {
    slot = &s_tx_queue[s_tx_tail];
    if (slot->queued && !slot->in_flight) {
      slot->in_flight = true;
      s_tx_busy = true;
    } else {
      slot = NULL;
    }
  }
  CRITICAL_SECTION_END();

  if (slot == NULL) {
    return;
  }

  uint8_t status = CDC_Transmit_FS(slot->data, slot->len);
  if (status != USBD_OK) {
    CRITICAL_SECTION_BEGIN();
    if (slot->queued && slot->in_flight) {
      slot->in_flight = false;
      s_tx_busy = false;
    }
    CRITICAL_SECTION_END();
  }
#endif
}

static bool vofa_transmit(const uint8_t *buf, uint16_t len) {
  if (buf == NULL || len == 0U || len > VOFA_TX_SLOT_SIZE) {
    return false;
  }

#if USART_OR_CDC == 0
  static uint8_t dma_buf[VOFA_TX_SLOT_SIZE];
  memcpy(dma_buf, buf, len);
  return HAL_UART_Transmit_DMA(&huart3, dma_buf, len) == HAL_OK;
#elif USART_OR_CDC == 1
  bool queued = false;
  CRITICAL_SECTION_BEGIN();
  if (s_tx_count < VOFA_TX_QUEUE_DEPTH) {
    VofaTxSlot *slot = &s_tx_queue[s_tx_head];
    memcpy(slot->data, buf, len);
    slot->len = len;
    slot->queued = true;
    slot->in_flight = false;
    s_tx_head = (uint8_t)((s_tx_head + 1U) % VOFA_TX_QUEUE_DEPTH);
    s_tx_count++;
    queued = true;
  }
  CRITICAL_SECTION_END();

  if (queued) {
    Vofa_TryStartTransmit();
  }
  return queued;
#endif
}

bool Vofa_QueueReceive(const uint8_t *buf, uint16_t len) {
  if (buf == NULL || len == 0U || len > VOFA_RX_SLOT_SIZE) {
    return false;
  }

  bool queued = false;
  CRITICAL_SECTION_BEGIN();
  if (s_rx_count < VOFA_RX_QUEUE_DEPTH) {
    VofaRxSlot *slot = &s_rx_queue[s_rx_head];
    memcpy(slot->data, buf, len);
    slot->len = len;
    s_rx_head = (uint8_t)((s_rx_head + 1U) % VOFA_RX_QUEUE_DEPTH);
    s_rx_count++;
    queued = true;
  } else {
    s_rx_overflow_count++;
  }
  CRITICAL_SECTION_END();
  return queued;
}

uint32_t Vofa_GetReceiveOverflowCount(void) {
  uint32_t count;
  CRITICAL_SECTION_BEGIN();
  count = s_rx_overflow_count;
  CRITICAL_SECTION_END();
  return count;
}

void Vofa_ReportScheduledSaveResult(bool succeeded) {
  bool report = false;

  CRITICAL_SECTION_BEGIN();
  if (s_save_flash_result_pending) {
    if (succeeded) {
      s_save_flash_result_pending = false;
      s_save_flash_failure_reported = false;
      report = true;
    } else if (!s_save_flash_failure_reported) {
      /* Param_ProcessScheduledSave keeps a failed request pending for retry.
       * Report the first failure honestly, then leave the request armed so a
       * later successful retry can still be acknowledged. */
      s_save_flash_failure_reported = true;
      report = true;
    }
  }
  CRITICAL_SECTION_END();

  if (report) {
    Studio_SendText(succeeded ? "ack=save_flash,succeeded"
                              : "ack=save_flash,retrying");
  }
}

static void Vofa_ProcessNextReceive(void) {
  uint8_t data[VOFA_RX_SLOT_SIZE];
  uint16_t len = 0U;

  CRITICAL_SECTION_BEGIN();
  if (s_rx_count > 0U) {
    VofaRxSlot *slot = &s_rx_queue[s_rx_tail];
    len = slot->len;
    memcpy(data, slot->data, len);
    slot->len = 0U;
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % VOFA_RX_QUEUE_DEPTH);
    s_rx_count--;
  }
  CRITICAL_SECTION_END();

  if (len > 0U) {
    vofa_Receive(data, len);
  }
}

void Vofa_Service(void) {
  bool request_boot_upgrade = false;
  uint32_t rx_overflow_count = 0U;

  if (!s_boot_upgrade_pending) {
    Vofa_ProcessNextReceive();
  }
  Vofa_TryStartTransmit();

  CRITICAL_SECTION_BEGIN();
  if (s_boot_upgrade_pending && !s_tx_busy && s_tx_count == 0U) {
    s_boot_upgrade_pending = false;
    request_boot_upgrade = true;
  }
  CRITICAL_SECTION_END();

  if (request_boot_upgrade) {
    Boot_RequestUpgrade();
    return;
  }
  if (s_boot_upgrade_pending) {
    return;
  }
  CRITICAL_SECTION_BEGIN();
  rx_overflow_count = s_rx_overflow_count;
  CRITICAL_SECTION_END();
  if (rx_overflow_count != s_rx_overflow_reported_count &&
      Studio_SendText("rx_overflow=1")) {
    /* Keep a newer counter value pending if another packet was dropped while
     * the notification was being queued. */
    s_rx_overflow_reported_count = rx_overflow_count;
  }
  Studio_ProcessParamSync();
}

void Vofa_OnTransmitComplete(void) {
#if USART_OR_CDC == 1
  CRITICAL_SECTION_BEGIN();
  if (s_tx_busy && s_tx_count > 0U) {
    VofaTxSlot *slot = &s_tx_queue[s_tx_tail];
    if (slot->queued && slot->in_flight) {
      slot->queued = false;
      slot->in_flight = false;
      slot->len = 0U;
      s_tx_tail = (uint8_t)((s_tx_tail + 1U) % VOFA_TX_QUEUE_DEPTH);
      s_tx_count--;
      s_tx_busy = false;
    }
  }
  CRITICAL_SECTION_END();
  Vofa_TryStartTransmit();
#endif
}
/* ============================================================================
 *   API
 * ============================================================================
 */
bool Studio_SendText(const char *text) {
  if (text == NULL)
    return false;
  size_t len = strlen(text);
  if (len > TEXT_LINE_MAX)
    return false;
  uint8_t text_buf[TEXT_LINE_MAX + 2];
  memcpy(text_buf, text, len);
  text_buf[len] = '\n';
  return vofa_transmit(text_buf, (uint16_t)(len + 1U));
}
bool Studio_SendTextf(const char *fmt, ...) {
  if (fmt == NULL) {
    return false;
  }
  char fmt_buf[TEXT_LINE_MAX + 1];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(fmt_buf, sizeof(fmt_buf), fmt, args);
  va_end(args);
  if (n > 0 && (size_t)n < sizeof(fmt_buf)) {
    return Studio_SendText(fmt_buf);
  }
  return false;
}
/* ============================================================================
 *   (FireWater )
 * ============================================================================
 */
void vofa_start(void) { Vofa_Packet(); }
void vofa_send_data(uint8_t num, float data) {
  (void)num; // num
  if (cnt + 4 < MAX_TXBUFFER_SIZE) {
    send_buf[cnt++] = byte0(data);
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
  }
}
void vofa_sendframetail(void) {
  if (cnt + 4 < MAX_TXBUFFER_SIZE) {
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;
    vofa_transmit(send_buf, cnt);
    cnt = 0;
  }
}
void Vofa_Packet(void) {
  vofa_send_data(0, motor_data.algo_input.Ia);
  vofa_send_data(1, motor_data.algo_input.Ib);
  vofa_send_data(2, motor_data.algo_input.Ic);
  vofa_send_data(3, motor_data.algo_output.Iq);
  vofa_send_data(4, motor_data.algo_output.Id);
  vofa_send_data(5, motor_data.algo_input.Iq_ref);
  vofa_send_data(6, motor_data.algo_input.Id_ref);
  vofa_send_data(7, motor_data.feedback.velocity);
  vofa_send_data(8, motor_data.feedback.position);
  vofa_send_data(9, motor_data.feedback.phase_angle);
  vofa_send_data(10, motor_data.feedback.temperature);
  vofa_send_data(11, motor_data.algo_input.Vbus);
  vofa_sendframetail();
}
/* ============================================================================
 *  Status tracking (forward declaration for Scope_Process)
 * ============================================================================
 */
static struct {
  int last_calib_step;
  bool last_calib_active;
  uint32_t last_fault_bits;
  bool version_sent;
  bool cogging_was_active;
  uint16_t last_cogging_step;
  uint8_t status_rate_div;
  uint8_t status_tick;
  bool scope_enabled;
} s_status = {
    .last_calib_step = -1,
    .last_calib_active = false,
    .last_fault_bits = 0,
    .version_sent = false,
    .cogging_was_active = false,
    .last_cogging_step = 0,
    .status_rate_div = 2,
    .status_tick = 0,
    .scope_enabled = true,
};

/* ============================================================================
 *  Scope  (ISR → Task)
 * ============================================================================
 */
static ScopeBuffer_t scope_buf;
void Scope_Init(void) {
  scope_buf.head = 0;
  scope_buf.tail = 0;
  cnt = 0U;
  CRITICAL_SECTION_BEGIN();
  memset(s_tx_queue, 0, sizeof(s_tx_queue));
  memset(s_rx_queue, 0, sizeof(s_rx_queue));
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_count = 0U;
  s_tx_head = 0U;
  s_tx_tail = 0U;
  s_tx_count = 0U;
  s_tx_busy = false;
  s_boot_upgrade_pending = false;
  s_save_flash_result_pending = false;
  s_save_flash_failure_reported = false;
  s_param_sync_active = false;
  s_param_sync_index = 0U;
  s_rx_overflow_count = 0U;
  s_rx_overflow_reported_count = 0U;
  CRITICAL_SECTION_END();
}
void Scope_Update(void) {
  uint16_t next_head = (scope_buf.head + 1) % SCOPE_BUFFER_SIZE;
  if (next_head == scope_buf.tail)
    return; //
  float *data = scope_buf.data[scope_buf.head];
  data[0] = motor_data.algo_input.Ia;
  data[1] = motor_data.algo_input.Ib;
  data[2] = motor_data.algo_input.Ic;
  data[3] = motor_data.algo_output.Iq;
  data[4] = motor_data.algo_output.Id;
  data[5] = motor_data.algo_input.Iq_ref;
  data[6] = motor_data.algo_input.Id_ref;
  data[11] = motor_data.algo_input.Vbus;
  data[7] = motor_data.feedback.velocity;
  data[8] = motor_data.feedback.position;
  data[9] = motor_data.feedback.phase_angle;
  data[10] = motor_data.feedback.temperature;
  scope_buf.head = next_head;
}
void Scope_Process(void) {
  Vofa_Service();
  if (scope_buf.tail != scope_buf.head) {
    if (s_status.scope_enabled) {
      float *data = scope_buf.data[scope_buf.tail];
      for (int i = 0; i < SCOPE_CHANNELS; i++) {
        vofa_send_data((uint8_t)i, data[i]);
      }
      vofa_sendframetail();
    }
    scope_buf.tail = (scope_buf.tail + 1) % SCOPE_BUFFER_SIZE;
  }
}
/* ============================================================================
 *  state ( → VectorStudio)
 * ============================================================================
 */
/* ── calibrationstate ── */
static int cs_state_to_step(CS_STATE cs) {
  if (cs >= CS_MOTOR_R_START && cs <= CS_MOTOR_R_END)
    return 1; // calibration
  if (cs >= CS_MOTOR_L_START && cs <= CS_MOTOR_L_END)
    return 2; // calibration
  if (cs >= CS_DIR_PP_START && cs <= CS_DIR_PP_END)
    return 3; // pole pairs/
  if (cs >= CS_ENCODER_START && cs <= CS_ENCODER_END)
    return 4; // encoder
  if (cs >= CS_FLUX_START && cs <= CS_FLUX_END)
    return 5; // flux
  if (cs == CS_REPORT_OFFSET_LUT)
    return 6; //
  return 0;   // IDLE
}
/* ── fault ── */
typedef struct {
  uint32_t bit;
  const char *code;
  const char *severity;
} FaultMap_t;
static const FaultMap_t s_fault_map[] = {
    {FAULT_OVER_TEMP, "OT", "CRITICAL"},
    {FAULT_DRIVER_CHIP, "DRV", "CRITICAL"},
    {FAULT_UNDER_VOLTAGE, "UV", "WARNING"},
    {FAULT_OVER_VOLTAGE, "OV", "CRITICAL"},
    {FAULT_CURRENT_B, "OC_B", "CRITICAL"},
    {FAULT_CURRENT_C, "OC_C", "CRITICAL"},
    {FAULT_ENCODER_LOSS, "ENC_LOSS", "CRITICAL"},
    {FAULT_ENCODER_UNCALIBRATED, "ENC_UNCAL", "WARNING"},
    {FAULT_HARDWARE_ID, "HW_ID", "CRITICAL"},
    {FAULT_POSITION_INIT, "POS_INIT", "WARNING"},
    {FAULT_STALL_OVERLOAD, "STALL", "CRITICAL"},
    {FAULT_ADC_STALE, "ADC_STALE", "CRITICAL"},
    {FAULT_CURRENT_A, "OC_A", "CRITICAL"},
};
#define FAULT_MAP_SIZE (sizeof(s_fault_map) / sizeof(FaultMap_t))

bool Studio_IsScopeEnabled(void) { return s_status.scope_enabled; }
void Studio_ReportVersion(void) {
  Studio_SendTextf("fw_version=%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
                   FW_VERSION_PATCH);
}
void Studio_ReportCalibStatus(void) {
  bool calibrating = (motor_data.state.State_Mode == STATE_MODE_DETECTING) &&
                     (motor_data.state.Sub_State != SUB_STATE_IDLE);
  if (calibrating) {
    int step = cs_state_to_step(motor_data.state.Cs_State);
    Studio_SendTextf("calib_step=%d", step);
  } else {
    Studio_SendText("calib_step=0");
  }
}
void Studio_ReportFaults(void) {
  uint32_t bits = Safety_GetActiveFaultBits();
  if (bits == 0) {
    Studio_SendText("fault_clear=all");
    return;
  }
  for (uint32_t i = 0; i < FAULT_MAP_SIZE; i++) {
    if (bits & s_fault_map[i].bit) {
      Studio_SendTextf("fault=%s,%s", s_fault_map[i].code,
                       s_fault_map[i].severity);
    }
  }
}
/**
 * @brief motorrunningstate
 * : state=mode:N,en:0/1,vel:X.X,pos:X.X,tq:X.X,iq:X.X,ladrc:0/1
 *  set_status_rate=N frequency
 */
static void Studio_ReportMotorStatus(void) {
  MotorState ds402 = StateMachine_GetState(&g_ds402_state_machine);
  int enabled = (ds402 == STATE_OPERATION_ENABLED) ? 1 : 0;
  int mode = (int)motor_data.state.Control_Mode;
  int ladrc = (motor_data.ladrc_enable >= 0.5f) ? 1 : 0;
  Studio_SendTextf(
      "state=mode:%d,en:%d,vel:%.2f,pos:%.3f,tq:%.3f,iq:%.3f,ladrc:%d", mode,
      enabled, motor_data.Controller.input_velocity,
      motor_data.Controller.input_position, motor_data.Controller.input_torque,
      motor_data.algo_output.Iq, ladrc);
}
void Studio_PeriodicUpdate(void) {
  /*  */
  if (!s_status.version_sent) {
    Studio_ReportVersion();
    s_status.version_sent = true;
  }
  /* ── calibrationstate ── */
  bool calibrating = (motor_data.state.State_Mode == STATE_MODE_DETECTING) &&
                     (motor_data.state.Sub_State != SUB_STATE_IDLE);
  if (calibrating) {
    int step = cs_state_to_step(motor_data.state.Cs_State);
    if (step != s_status.last_calib_step) {
      s_status.last_calib_step = step;
      Studio_SendTextf("calib_step=%d", step);
    }
    s_status.last_calib_active = true;
  } else if (s_status.last_calib_active) {
    // calibration
    s_status.last_calib_active = false;
    s_status.last_calib_step = -1;
    // check (State_Mode  RUNNING = )
    if (motor_data.state.State_Mode == STATE_MODE_RUNNING) {
      Studio_SendText("calib_done=1");
      // calibrationdoneparam
      Studio_SendTextf("param=%d,%.6g", 0x2000, motor_data.parameters.Rs);
      Studio_SendTextf("param=%d,%.6g", 0x2001, motor_data.parameters.Ls);
      Studio_SendTextf("param=%d,%.6g", 0x2002, motor_data.parameters.flux);
      Studio_SendTextf("param=%d,%d", 0x2003,
                       (int)motor_data.parameters.pole_pairs);
    } else if (motor_data.state.State_Mode == STATE_MODE_GUARD) {
      Studio_SendText("calib_error=fault_during_calibration");
    } else {
      Studio_SendText("calib_done=1");
    }
  }
  /* ── faultstate ── */
  uint32_t fault_bits = Safety_GetActiveFaultBits();
  uint32_t new_faults = fault_bits & ~s_status.last_fault_bits;
  uint32_t cleared_faults = s_status.last_fault_bits & ~fault_bits;
  // fault
  if (new_faults) {
    for (uint32_t i = 0; i < FAULT_MAP_SIZE; i++) {
      if (new_faults & s_fault_map[i].bit) {
        Studio_SendTextf("fault=%s,%s", s_fault_map[i].code,
                         s_fault_map[i].severity);
      }
    }
  }
  // fault
  if (cleared_faults && fault_bits == 0) {
    Studio_SendText("fault_clear=all");
  }
  s_status.last_fault_bits = fault_bits;
  /* ── calibration ── */
  bool cogging_active = CoggingComp_IsCalibrating();
  if (cogging_active) {
    uint16_t step = CoggingComp_GetCalibStep();
    //  10  ()
    if (step != s_status.last_cogging_step && (step % 10 == 0 || step == 0)) {
      s_status.last_cogging_step = step;
      Studio_SendTextf("cogging_step=%u,%u", (unsigned)step,
                       (unsigned)COGGING_MAP_SIZE);
    }
    s_status.cogging_was_active = true;
  } else if (s_status.cogging_was_active) {
    s_status.cogging_was_active = false;
    s_status.last_cogging_step = 0;
    if (CoggingComp_IsValid()) {
      Studio_SendText("cogging_done=1");
    } else {
      Studio_SendText("cogging_error=aborted");
    }
  }
  /* ── periodmotorstate ── */
  if (s_status.status_rate_div > 0) {
    s_status.status_tick++;
    if (s_status.status_tick >= s_status.status_rate_div) {
      s_status.status_tick = 0;
      Studio_ReportMotorStatus();
    }
  }
}
/* ============================================================================
 *
 * ============================================================================
 */
static const char *vofa_skip_space(const char *text) {
  while (text != NULL &&
         (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n')) {
    text++;
  }
  return text;
}

static bool vofa_cmd_matches(const char *recv_str, const char *command) {
  if (recv_str == NULL || command == NULL) {
    return false;
  }

  size_t command_len = strlen(command);
  if (strncmp(recv_str, command, command_len) != 0) {
    return false;
  }
  if (command_len > 0U && command[command_len - 1U] == '=') {
    return true;
  }

  const char *suffix = vofa_skip_space(recv_str + command_len);
  return suffix != NULL && *suffix == '\0';
}

static bool vofa_token_ended(const char *text) {
  text = vofa_skip_space(text);
  return text != NULL && *text == '\0';
}

static bool vofa_parse_float_token(const char *text, const char **end,
                                   float *value) {
  if (text == NULL || value == NULL) {
    return false;
  }

  const char *cursor = vofa_skip_space(text);
  bool negative = false;
  if (*cursor == '+' || *cursor == '-') {
    negative = *cursor == '-';
    cursor++;
  }

  float parsed = 0.0f;
  bool has_digit = false;
  while (*cursor >= '0' && *cursor <= '9') {
    has_digit = true;
    parsed = parsed * 10.0f + (float)(*cursor - '0');
    if (!isfinite(parsed)) {
      return false;
    }
    cursor++;
  }

  if (*cursor == '.') {
    float place = 0.1f;
    cursor++;
    while (*cursor >= '0' && *cursor <= '9') {
      has_digit = true;
      parsed += (float)(*cursor - '0') * place;
      place *= 0.1f;
      cursor++;
    }
  }
  if (!has_digit) {
    return false;
  }

  if (*cursor == 'e' || *cursor == 'E') {
    bool exponent_negative = false;
    uint32_t exponent = 0U;
    cursor++;
    if (*cursor == '+' || *cursor == '-') {
      exponent_negative = *cursor == '-';
      cursor++;
    }
    if (*cursor < '0' || *cursor > '9') {
      return false;
    }
    while (*cursor >= '0' && *cursor <= '9') {
      if (exponent <= 1000U) {
        exponent = exponent * 10U + (uint32_t)(*cursor - '0');
      }
      cursor++;
    }

    if (exponent > 80U) {
      if (exponent_negative || parsed == 0.0f) {
        parsed = 0.0f;
        exponent = 0U;
      } else {
        return false;
      }
    }
    while (exponent-- > 0U) {
      parsed *= exponent_negative ? 0.1f : 10.0f;
      if (!isfinite(parsed)) {
        return false;
      }
    }
  }

  parsed = negative ? -parsed : parsed;
  if (!isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  if (end != NULL) {
    *end = cursor;
  }
  return true;
}

static bool vofa_parse_int_token(const char *text, const char **end,
                                 int *value) {
  if (text == NULL || value == NULL) {
    return false;
  }
  const char *cursor = vofa_skip_space(text);
  bool negative = false;
  if (*cursor == '+' || *cursor == '-') {
    negative = *cursor == '-';
    cursor++;
  }
  if (*cursor < '0' || *cursor > '9') {
    return false;
  }

  uint32_t magnitude = 0U;
  const uint32_t limit =
      negative ? (uint32_t)INT32_MAX + 1U : (uint32_t)INT32_MAX;
  while (*cursor >= '0' && *cursor <= '9') {
    uint32_t digit = (uint32_t)(*cursor - '0');
    if (magnitude > (limit - digit) / 10U) {
      return false;
    }
    magnitude = magnitude * 10U + digit;
    cursor++;
  }
  *value = (negative && magnitude == (uint32_t)INT32_MAX + 1U)
               ? INT32_MIN
               : (negative ? -(int)magnitude : (int)magnitude);
  if (end != NULL) {
    *end = cursor;
  }
  return true;
}

static bool vofa_cmd_parse_float_checked(const char *recv_str, const char *arg,
                                         float *value) {
  const char *end = NULL;
  return vofa_cmd_matches(recv_str, arg) &&
         vofa_parse_float_token(recv_str + strlen(arg), &end, value) &&
         vofa_token_ended(end);
}

static bool vofa_cmd_parse_int_checked(const char *recv_str, const char *arg,
                                       int *value) {
  const char *end = NULL;
  return vofa_cmd_matches(recv_str, arg) &&
         vofa_parse_int_token(recv_str + strlen(arg), &end, value) &&
         vofa_token_ended(end);
}

static bool vofa_cmd_parse(const char *recv_str, const char *arg,
                           float *value) {
  if (vofa_cmd_parse_float_checked(recv_str, arg, value)) {
    return true;
  }
  Studio_SendTextf("cmd_err=%sINVALID_VALUE", arg);
  return false;
}

static bool vofa_cmd_parse_int(const char *recv_str, const char *arg,
                               int *value) {
  if (vofa_cmd_parse_int_checked(recv_str, arg, value)) {
    return true;
  }
  Studio_SendTextf("cmd_err=%sINVALID_VALUE", arg);
  return false;
}

static const char *Studio_ParamErrorName(ParamResult result) {
  switch (result) {
  case PARAM_ERR_INVALID_INDEX:
    return "NOT_FOUND";
  case PARAM_ERR_INVALID_TYPE:
    return "INVALID_TYPE";
  case PARAM_ERR_READONLY:
    return "READONLY";
  case PARAM_ERR_OUT_OF_RANGE:
    return "OUT_OF_RANGE";
  case PARAM_ERR_STORAGE:
    return "STORAGE";
  case PARAM_ERR_NULL_PTR:
  default:
    return "INTERNAL";
  }
}

static ParamResult Studio_WriteParameter(uint16_t index, float value,
                                         bool report_success) {
  ParamResult result = Param_WriteFromFloat(index, value);
  if (result != PARAM_OK) {
    Studio_SendTextf("param_err=%u,%s", (unsigned)index,
                     Studio_ParamErrorName(result));
  } else if (report_success) {
    float applied = 0.0f;
    if (Param_ReadAsFloat(index, &applied) == PARAM_OK) {
      Studio_SendTextf("param=%u,%.9g", (unsigned)index, applied);
    }
  }
  return result;
}

static void Studio_ProcessParamSync(void) {
  while (s_param_sync_active) {
    const ParamEntry *table = ParamTable_GetTable();
    uint32_t count = ParamTable_GetCount();
    uint32_t index = s_param_sync_index;

    if (table == NULL || index >= count) {
      if (Studio_SendText("param_sync_done")) {
        CRITICAL_SECTION_BEGIN();
        s_param_sync_active = false;
        s_param_sync_index = 0U;
        CRITICAL_SECTION_END();
      }
      return;
    }

    const ParamEntry *entry = &table[index];
    float value = 0.0f;
    bool queued;
    if (Param_ReadAsFloat(entry->index, &value) == PARAM_OK) {
      queued = Studio_SendTextf("param=%u,%.9g", (unsigned)entry->index, value);
    } else {
      queued = Studio_SendTextf("param=%u,ERR", (unsigned)entry->index);
    }
    if (!queued) {
      return;
    }

    CRITICAL_SECTION_BEGIN();
    if (s_param_sync_active && s_param_sync_index == index) {
      s_param_sync_index++;
    }
    CRITICAL_SECTION_END();
  }
}
/* ============================================================================
 *   (VectorStudio → )
 * ============================================================================
 */
void vofa_Receive(uint8_t *buf, uint16_t len) {
  if (buf == NULL || len == 0U) {
    return;
  }
  if (len >= MAX_RXBUFFER_SIZE)
    len = MAX_RXBUFFER_SIZE - 1;
  memcpy(receive_buf, buf, len);
  receive_buf[len] = '\0';
  char *recvStr = (char *)receive_buf;
  /* ──  ── */
  if (vofa_cmd_matches(recvStr, "handshake=studio")) {
    Studio_SendTextf("ack=studio,%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
                     FW_VERSION_PATCH);
    s_status.version_sent = true;
    return;
  }
  if (vofa_cmd_matches(recvStr, "get_version")) {
    Studio_ReportVersion();
    return;
  }
  if (vofa_cmd_matches(recvStr, "get_param=")) {
    int idx = 0;
    float value = 0.0f;
    if (!vofa_cmd_parse_int_checked(recvStr, "get_param=", &idx)) {
      Studio_SendText("param_err=INVALID_FORMAT");
      return;
    }
    ParamResult result = (idx >= 0 && idx <= UINT16_MAX)
                             ? Param_ReadAsFloat((uint16_t)idx, &value)
                             : PARAM_ERR_INVALID_INDEX;
    if (result == PARAM_OK) {
      Studio_SendTextf("param=%d,%.9g", idx, value);
    } else {
      Studio_SendTextf("param_err=%d,%s", idx, Studio_ParamErrorName(result));
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_param=")) {
    const char *payload = recvStr + strlen("set_param=");
    const char *cursor = payload;
    int idx = 0;
    float val = 0.0f;
    bool valid = vofa_parse_int_token(cursor, &cursor, &idx);
    cursor = vofa_skip_space(cursor);
    if (valid && cursor != NULL && *cursor == ',') {
      cursor++;
      valid = vofa_parse_float_token(cursor, &cursor, &val) &&
              vofa_token_ended(cursor);
    } else {
      valid = false;
    }
    if (valid && idx >= 0 && idx <= UINT16_MAX) {
      (void)Studio_WriteParameter((uint16_t)idx, val, true);
    } else {
      Studio_SendTextf("param_err=%d,INVALID_FORMAT", idx);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "read_all_params")) {
    CRITICAL_SECTION_BEGIN();
    s_param_sync_index = 0U;
    s_param_sync_active = true;
    CRITICAL_SECTION_END();
    return;
  }
  if (vofa_cmd_matches(recvStr, "save_flash=1")) {
    Param_ScheduleSave();
    CRITICAL_SECTION_BEGIN();
    s_save_flash_result_pending = true;
    s_save_flash_failure_reported = false;
    CRITICAL_SECTION_END();
    Studio_SendText("ack=save_flash,queued");
    return;
  }
  /* ── motor ── */
  if (vofa_cmd_matches(recvStr, "motor_enable=")) {
    float motor_enable = 0.0f;
    if (!vofa_cmd_parse(recvStr, "motor_enable=", &motor_enable)) {
      return;
    }
    MotorCommand cmd = {0};
    cmd.has_enable_command = true;
    cmd.enable_motor = motor_enable > 0.5f;
    Executor_ProcessCommand(&cmd);
    return;
  }
  if (vofa_cmd_matches(recvStr, "calib=")) {
    float calib_enable = 0.0f;
    if (!vofa_cmd_parse(recvStr, "calib=", &calib_enable)) {
      return;
    }
    if (calib_enable > 0.5f) {
      s_status.last_calib_active = false; //
      s_status.last_calib_step = -1;
      StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
      Studio_SendText("calib_step=0");
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "clear_fault=1")) {
    if (Motor_ClearFaults(&motor_data)) {
      s_status.last_fault_bits = 0;
      Studio_SendText("fault_clear=all");
    } else {
      Studio_SendText("fault_clear=unsafe");
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_zero=1")) {
    bool zeroed = false;
    CRITICAL_SECTION_BEGIN();
    if (MHAL_Encoder_ZeroPosition() == 0) {
      motor_data.feedback.position = 0.0f;
      motor_data.Controller.input_position = 0.0f;
      motor_data.Controller.pos_setpoint = 0.0f;
      motor_data.Controller.mit_pos_des = 0.0f;
      motor_data.Controller.input_updated = true;
      zeroed = true;
    }
    CRITICAL_SECTION_END();
    if (zeroed) {
      Studio_SendText("ack=zero_set");
    } else {
      Studio_SendText("ack=zero_failed");
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_ctrl_mode=")) {
    int ctrlModeInt = 0;
    if (vofa_cmd_parse_int_checked(recvStr, "set_ctrl_mode=", &ctrlModeInt) &&
        ctrlModeInt >= CONTROL_MODE_OPEN && ctrlModeInt <= CONTROL_MODE_MIT) {
      MotorCommand cmd = {0};
      cmd.has_control_mode = true;
      cmd.control_mode = (uint8_t)ctrlModeInt;
      Executor_ProcessCommand(&cmd);
    } else {
      Studio_SendText("param_err=ctrl_mode,INVALID_VALUE");
    }
    return;
  }
  /* ──  ── */
  if (vofa_cmd_matches(recvStr, "set_Iq=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_Iq=", &value)) {
      MotorCommand cmd = {0};
      cmd.has_iq_ref = true;
      cmd.iq_ref = value;
      Executor_ProcessCommand(&cmd);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_Id=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_Id=", &value)) {
      MotorCommand cmd = {0};
      cmd.has_id_ref = true;
      cmd.id_ref = value;
      Executor_ProcessCommand(&cmd);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_torque=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_torque=", &value)) {
      MotorCommand cmd = {0};
      cmd.has_torque_ref = true;
      cmd.iq_ref = value;
      Executor_ProcessCommand(&cmd);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_vel=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_vel=", &value)) {
      MotorCommand cmd = {0};
      /* VOFA historically exposes the controller's native turn/s unit.
       * Convert at the USB boundary because Executor_ProcessCommand accepts
       * protocol-facing SI radians per second. */
      cmd.has_velocity_ref = true;
      cmd.speed_ref = Protocol_TurnsToRadians(value);
      Executor_ProcessCommand(&cmd);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_pos=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_pos=", &value)) {
      MotorCommand cmd = {0};
      /* Preserve the legacy VOFA turn unit while using the SI executor API. */
      cmd.has_position_ref = true;
      cmd.position_ref = Protocol_TurnsToRadians(value);
      Executor_ProcessCommand(&cmd);
    }
    return;
  }
  /* ── PID  ── */
  if (vofa_cmd_matches(recvStr, "set_current_ctrl_bw=")) {
    float bandwidth = 0.0f;
    if (!vofa_cmd_parse(recvStr, "set_current_ctrl_bw=", &bandwidth)) {
      return;
    }
    if (isfinite(bandwidth) && bandwidth >= 100.0f && bandwidth <= 2000.0f) {
      CRITICAL_SECTION_BEGIN();
      motor_data.Controller.current_ctrl_bandwidth = (int)bandwidth;
      CurrentLoop_UpdateGain(&motor_data);
      CRITICAL_SECTION_END();
    } else {
      Studio_SendText("param_err=current_ctrl_bw,OUT_OF_RANGE");
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_vel_kp=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_vel_kp=", &value)) {
      (void)Studio_WriteParameter(PARAM_SPD_KP, value, false);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_vel_ki=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_vel_ki=", &value)) {
      (void)Studio_WriteParameter(PARAM_SPD_KI, value, false);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_pos_kp=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_pos_kp=", &value)) {
      (void)Studio_WriteParameter(PARAM_POS_KP, value, false);
    }
    return;
  }
  /* ── LADRC speed/velocityparam ── */
  if (vofa_cmd_matches(recvStr, "set_ladrc_en=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_ladrc_en=", &value)) {
      (void)Studio_WriteParameter(PARAM_LADRC_ENABLE, value, false);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_ladrc_wo=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_ladrc_wo=", &value)) {
      (void)Studio_WriteParameter(PARAM_LADRC_OMEGA_O, value, false);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_ladrc_wc=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_ladrc_wc=", &value)) {
      (void)Studio_WriteParameter(PARAM_LADRC_OMEGA_C, value, false);
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_ladrc_b0=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_ladrc_b0=", &value)) {
      (void)Studio_WriteParameter(PARAM_LADRC_B0, value, false);
    }
    return;
  }
  /* ──  ── */
  if (vofa_cmd_matches(recvStr, "set_cogging_calib=1")) {
    if (Studio_WriteParameter(PARAM_COGGING_CALIB, 1.0f, false) == PARAM_OK) {
      Studio_SendText("ack=cogging_calib_started");
    }
    return;
  }
  if (vofa_cmd_matches(recvStr, "set_cogging_enable=")) {
    float value = 0.0f;
    if (vofa_cmd_parse(recvStr, "set_cogging_enable=", &value)) {
      (void)Studio_WriteParameter(PARAM_COGGING_EN, value, false);
    }
    return;
  }
  /* ──  ── */
  // setstatefrequency: set_status_rate=N (0=, 1=10Hz, 2=5Hz, 5=2Hz, 10=1Hz)
  if (vofa_cmd_matches(recvStr, "set_status_rate=")) {
    int rate = 0;
    if (!vofa_cmd_parse_int(recvStr, "set_status_rate=", &rate)) {
      return;
    }
    s_status.status_rate_div = (uint8_t)CLAMP(rate, 0, 100);
    s_status.status_tick = 0;
    Studio_SendTextf("ack=status_rate,%d", (int)s_status.status_rate_div);
    return;
  }
  //  (scope) enable/: set_scope_enable=0/1
  if (vofa_cmd_matches(recvStr, "set_scope_enable=")) {
    float en = 0.0f;
    if (!vofa_cmd_parse(recvStr, "set_scope_enable=", &en)) {
      return;
    }
    s_status.scope_enabled = (en > 0.5f);
    Studio_SendTextf("ack=scope_enable,%d", s_status.scope_enabled ? 1 : 0);
    return;
  }
  // state ()
  if (vofa_cmd_matches(recvStr, "get_status")) {
    Studio_ReportMotorStatus();
    return;
  }
  /* ── CAN  () ── */
  if (vofa_cmd_matches(recvStr, "scan_bus=1")) {
    /* 单节点设备直接上报自身 CAN ID（无总线扫描硬件支持）；
     * 如需多节点枚举，应通过 CAN 广播帧由各节点自报。 */
    Studio_SendText("bus_scan_start");
    Studio_SendTextf("bus_node=%d,online,%.0f", g_can_id,
                     motor_data.feedback.temperature);
    Studio_SendText("bus_scan_done");
    return;
  }
  /* ── OTA 升级命令 ── */
  if (vofa_cmd_matches(recvStr, "boot_enter")) {
#if defined(BOARD_XSTAR)
    Studio_SendText("boot_ack,2,unsupported");
    return;
#else
    /* 先禁用电机 */
    StateMachine_RequestState(&g_ds402_state_machine, STATE_SWITCH_ON_DISABLED);
    /* ACK 完成后由 Vofa_Service 在任务上下文请求重启。 */
    if (Studio_SendText("boot_ack,0,entering_bootloader")) {
      CRITICAL_SECTION_BEGIN();
      s_boot_upgrade_pending = true;
      CRITICAL_SECTION_END();
    }
    return;
#endif
  }
}
