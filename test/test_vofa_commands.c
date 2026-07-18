// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "protocol_types.h"

#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define APP_RX_DATA_SIZE 1024U
#define APP_TX_DATA_SIZE 1024U
#define UNUSED(x) (void)(x)
#define CDC_SEND_ENCAPSULATED_COMMAND 0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE 0x01U
#define CDC_SET_COMM_FEATURE 0x02U
#define CDC_GET_COMM_FEATURE 0x03U
#define CDC_CLEAR_COMM_FEATURE 0x04U
#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
#define CDC_SEND_BREAK 0x23U

typedef struct {
  uint8_t TxState;
} USBD_CDC_HandleTypeDef;

typedef struct {
  void *pClassData;
} USBD_HandleTypeDef;

typedef struct {
  int8_t (*Init)(void);
  int8_t (*DeInit)(void);
  int8_t (*Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (*Receive)(uint8_t *pbuf, uint32_t *Len);
  int8_t (*TransmitCplt)(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
} USBD_CDC_ItfTypeDef;

MOTOR_DATA motor_data;
StateMachine g_ds402_state_machine;
uint8_t g_can_id = 7U;
uint8_t g_can_baudrate;
uint8_t g_protocol_type;
uint32_t g_can_timeout_ms;
uint8_t g_zero_sta;
float g_add_offset;
uint8_t g_damper_enable;
uint8_t g_run_mode;

static USBD_CDC_HandleTypeDef s_hcdc;
USBD_HandleTypeDef hUsbDeviceFS = {.pClassData = &s_hcdc};

static const uint8_t *s_pending_tx_ptr;
static uint16_t s_pending_tx_len;
static uint8_t s_transmit_result;
static unsigned s_set_tx_count;
static unsigned s_transmit_packet_count;
static unsigned s_delay_call_count;
static unsigned s_boot_request_count;
static unsigned s_executor_count;
static MotorCommand s_last_executor_cmd;
static unsigned s_save_schedule_count;
static unsigned s_save_begin_count;
static unsigned s_save_commit_count;
static unsigned s_save_cancel_count;
static bool s_save_request_available;
static bool s_save_commit_reports_success;
static bool s_motor_clear_result;
static char s_wire_log[2048];

void USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff,
                          uint16_t length) {
  (void)pdev;
  s_pending_tx_ptr = pbuff;
  s_pending_tx_len = length;
  s_set_tx_count++;
}

uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev) {
  (void)pdev;
  s_transmit_packet_count++;
  if (s_transmit_result == USBD_OK) {
    s_hcdc.TxState = 1U;
    size_t used = strlen(s_wire_log);
    size_t room = sizeof(s_wire_log) - used - 1U;
    size_t copy = s_pending_tx_len < room ? s_pending_tx_len : room;
    memcpy(&s_wire_log[used], s_pending_tx_ptr, copy);
    s_wire_log[used + copy] = '\0';
  }
  return s_transmit_result;
}

void USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff) {
  (void)pdev;
  (void)pbuff;
}

uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev) {
  (void)pdev;
  return USBD_OK;
}

static uint16_t s_param_index;
static float s_param_value;
static ParamResult s_param_write_result = PARAM_OK;
static ParamResult s_param_read_result = PARAM_OK;
static unsigned s_write_from_float_count;
static unsigned s_read_as_float_count;
static float s_backing_float;
static ParamEntry s_sync_entries[10];
static uint32_t s_sync_entry_count;

static ParamEntry s_float_entry = {.index = PARAM_MOTOR_RS,
                                   .type = PARAM_TYPE_FLOAT,
                                   .access = PARAM_ACCESS_RW,
                                   .ptr = &s_backing_float,
                                   .min = -1000.0f,
                                   .max = 1000.0f};

void Vofa_ReportScheduledSaveResult(bool succeeded);

const ParamEntry *ParamTable_Find(uint16_t index) {
  s_float_entry.index = index;
  s_float_entry.ptr = &s_backing_float;
  s_float_entry.type = PARAM_TYPE_FLOAT;
  s_float_entry.access = PARAM_ACCESS_RW;
  s_float_entry.min = -1000.0f;
  s_float_entry.max = 1000.0f;
  return &s_float_entry;
}

uint32_t ParamTable_GetCount(void) { return s_sync_entry_count; }
const ParamEntry *ParamTable_GetTable(void) {
  return s_sync_entry_count > 0U ? s_sync_entries : NULL;
}
void ParamTable_Init(void) {}

ParamResult Param_WriteFromFloat(uint16_t index, float value) {
  s_write_from_float_count++;
  s_param_index = index;
  s_param_value = value;
  if (s_param_write_result == PARAM_OK) {
    s_backing_float = value;
  }
  return s_param_write_result;
}

ParamResult Param_ReadAsFloat(uint16_t index, float *value) {
  s_read_as_float_count++;
  s_param_index = index;
  if (s_param_read_result == PARAM_OK && value != NULL) {
    *value = s_param_value;
  }
  return s_param_read_result;
}

void Param_ScheduleSave(void) { s_save_schedule_count++; }
bool CmdService_BeginScheduledSave(void) {
  if (!s_save_request_available ||
      g_ds402_state_machine.current_state == STATE_OPERATION_ENABLED ||
      g_ds402_state_machine.current_state == STATE_CALIBRATING) {
    return false;
  }
  s_save_begin_count++;
  return true;
}

void CmdService_CommitScheduledSave(void) {
  s_save_commit_count++;
  s_save_schedule_count++;
  if (s_save_commit_reports_success) {
    Vofa_ReportScheduledSaveResult(true);
  }
}

void CmdService_CancelScheduledSave(void) { s_save_cancel_count++; }

bool CmdService_RequestScheduledSave(void) {
  if (!CmdService_BeginScheduledSave()) {
    return false;
  }
  CmdService_CommitScheduledSave();
  return true;
}

void CurrentLoop_UpdateGain(MOTOR_DATA *motor) { (void)motor; }
void LADRC_Init(LADRC_State_t *state, const LADRC_Config_t *config) {
  (void)state;
  (void)config;
}
void LADRC_Reset(LADRC_State_t *state) { (void)state; }
void LADRC_UpdateGains(LADRC_State_t *state, const LADRC_Config_t *config) {
  (void)state;
  (void)config;
}
void PID_clear(PidTypeDef *pid) { (void)pid; }
bool CoggingComp_IsCalibrating(void) { return false; }
bool CoggingComp_IsValid(void) { return false; }
uint16_t CoggingComp_GetCalibStep(void) { return 0U; }
uint32_t Safety_GetActiveFaultBits(void) { return 0U; }
MotorState StateMachine_GetState(const StateMachine *sm) {
  (void)sm;
  return STATE_SWITCH_ON_DISABLED;
}
bool StateMachine_RequestState(StateMachine *sm, MotorState target_state) {
  (void)sm;
  (void)target_state;
  return true;
}
bool Motor_ClearFaults(MOTOR_DATA *motor) {
  (void)motor;
  return s_motor_clear_result;
}
int MHAL_Encoder_ZeroPosition(void) { return 0; }
void HAL_Delay(uint32_t ms) {
  (void)ms;
  s_delay_call_count++;
}
void Boot_RequestUpgrade(void) { s_boot_request_count++; }
void Executor_ProcessCommand(const MotorCommand *cmd) {
  s_executor_count++;
  if (cmd != NULL) {
    s_last_executor_cmd = *cmd;
  }
}

#ifndef __USART_H__
#define __USART_H__
#endif
#ifndef CORE_ADC_H
#define CORE_ADC_H
#endif
#ifndef MT6816_ENCODER_H
#define MT6816_ENCODER_H
#endif
#ifndef BOOTLOADER_H
#define BOOTLOADER_H
#endif
#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H
typedef struct {
  uint32_t words[8];
} AppHeader_t;
#endif
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__
uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);
#endif

#include "../Lib/USB_Device/App/usbd_cdc_if.c"
#include "../Src/UI/vofa/vofa.c"

static void ResetHarness(void) {
  memset(&motor_data, 0, sizeof(motor_data));
  memset(&g_ds402_state_machine, 0, sizeof(g_ds402_state_machine));
  memset(s_wire_log, 0, sizeof(s_wire_log));
  s_pending_tx_ptr = NULL;
  s_pending_tx_len = 0U;
  s_transmit_result = USBD_OK;
  s_set_tx_count = 0U;
  s_transmit_packet_count = 0U;
  s_delay_call_count = 0U;
  s_boot_request_count = 0U;
  s_executor_count = 0U;
  memset(&s_last_executor_cmd, 0, sizeof(s_last_executor_cmd));
  s_save_schedule_count = 0U;
  s_save_begin_count = 0U;
  s_save_commit_count = 0U;
  s_save_cancel_count = 0U;
  s_save_request_available = true;
  s_save_commit_reports_success = false;
  s_motor_clear_result = true;
  s_hcdc.TxState = 0U;
  s_param_index = 0U;
  s_param_value = 0.0f;
  s_param_write_result = PARAM_OK;
  s_param_read_result = PARAM_OK;
  s_write_from_float_count = 0U;
  s_read_as_float_count = 0U;
  s_backing_float = 0.0f;
  s_sync_entry_count = 0U;
  memset(s_sync_entries, 0, sizeof(s_sync_entries));
  Scope_Init();
}

static void CompleteTx(void) {
  uint32_t len = s_pending_tx_len;
  s_hcdc.TxState = 0U;
  (void)USBD_Interface_fops_FS.TransmitCplt((uint8_t *)s_pending_tx_ptr, &len,
                                            0U);
}

static int Expect(bool condition, const char *message) {
  if (!condition) {
    printf("FAIL %s\n", message);
    return 1;
  }
  return 0;
}

static unsigned CountWireOccurrences(const char *needle) {
  unsigned count = 0U;
  const char *cursor = s_wire_log;
  while ((cursor = strstr(cursor, needle)) != NULL) {
    count++;
    cursor += strlen(needle);
  }
  return count;
}

static int TestUsbReceiveDefersCommandToTaskService(void) {
  uint8_t command[] = "set_param=8192,12.5";
  uint32_t len = (uint32_t)strlen((const char *)command);
  ResetHarness();

  if (Expect(USBD_Interface_fops_FS.Receive(command, &len) == USBD_OK,
             "USB receive callback rejected a valid command")) {
    return 1;
  }
  if (Expect(s_write_from_float_count == 0U,
             "USB receive callback executed parameter business logic")) {
    return 1;
  }
  if (Expect(s_rx_count == 1U,
             "USB receive callback did not queue the command")) {
    return 1;
  }

  Vofa_Service();
  return Expect(s_write_from_float_count == 1U &&
                    s_param_index == PARAM_MOTOR_RS && s_param_value == 12.5f,
                "task service did not execute the queued USB command");
}

static int TestSetParamUsesWireWriteApi(void) {
  uint8_t cmd[] = "set_param=8192,12.5";
  ResetHarness();
  vofa_Receive(cmd, (uint16_t)strlen((const char *)cmd));
  if (Expect(s_write_from_float_count == 1U,
             "set_param did not call Param_WriteFromFloat")) {
    return 1;
  }
  if (Expect(s_param_index == PARAM_MOTOR_RS && s_param_value == 12.5f,
             "set_param passed the wrong index or value")) {
    return 1;
  }
  return Expect(strstr(s_wire_log, "param=8192,12.5") != NULL,
                "set_param did not acknowledge the accepted value");
}

static int TestSetParamRejectsWireWriteFailure(void) {
  uint8_t cmd[] = "set_param=8192,12.5";
  ResetHarness();
  s_param_write_result = PARAM_ERR_OUT_OF_RANGE;
  vofa_Receive(cmd, (uint16_t)strlen((const char *)cmd));
  if (Expect(s_write_from_float_count == 1U,
             "set_param failure path did not call Param_WriteFromFloat")) {
    return 1;
  }
  if (Expect(s_backing_float == 0.0f,
             "set_param mutated backing storage after API failure")) {
    return 1;
  }
  return Expect(strstr(s_wire_log, "param_err=8192") != NULL,
                "set_param did not report the write API failure");
}

static int TestSetParamStrictDecimalParsing(void) {
  uint8_t scientific[] = "set_param=8192,-1.25e1";
  uint8_t trailing_junk[] = "set_param=8192,12.5junk";
  uint8_t overflow[] = "set_param=8192,1e100";

  ResetHarness();
  vofa_Receive(scientific, (uint16_t)strlen((const char *)scientific));
  if (Expect(s_write_from_float_count == 1U && s_param_value == -12.5f,
             "scientific decimal input was not parsed correctly")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(trailing_junk, (uint16_t)strlen((const char *)trailing_junk));
  if (Expect(s_write_from_float_count == 0U,
             "trailing junk reached the parameter write API")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(overflow, (uint16_t)strlen((const char *)overflow));
  return Expect(s_write_from_float_count == 0U,
                "overflowing decimal reached the parameter write API");
}

static int TestMalformedCommandsDoNotMutateControlState(void) {
  uint8_t bad_mode[] = "set_ctrl_mode=not-a-number";
  uint8_t bad_position[] = "set_pos=12junk";

  ResetHarness();
  motor_data.state.Control_Mode = CONTROL_MODE_POSITION;
  vofa_Receive(bad_mode, (uint16_t)strlen((const char *)bad_mode));
  if (Expect(motor_data.state.Control_Mode == CONTROL_MODE_POSITION,
             "malformed mode command changed the control mode")) {
    return 1;
  }

  ResetHarness();
  motor_data.Controller.input_position = 0.75f;
  motor_data.Controller.input_updated = false;
  vofa_Receive(bad_position, (uint16_t)strlen((const char *)bad_position));
  if (Expect(motor_data.Controller.input_position == 0.75f,
             "malformed position command changed the setpoint")) {
    return 1;
  }
  return Expect(!motor_data.Controller.input_updated,
                "malformed position command published an update");
}

static int TestVofaMotorCommandsUseExecutor(void) {
  uint8_t enable[] = "motor_enable=1";
  uint8_t mode[] = "set_ctrl_mode=0";
  uint8_t iq[] = "set_Iq=1.25";
  uint8_t id[] = "set_Id=-0.5";
  uint8_t torque[] = "set_torque=2.5";
  uint8_t velocity[] = "set_vel=3.5";
  uint8_t position[] = "set_pos=4.5";

  ResetHarness();
  vofa_Receive(enable, (uint16_t)strlen((const char *)enable));
  if (Expect(s_executor_count == 1U &&
                 s_last_executor_cmd.has_enable_command &&
                 s_last_executor_cmd.enable_motor,
             "motor_enable did not publish through executor")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(mode, (uint16_t)strlen((const char *)mode));
  if (Expect(s_executor_count == 1U && s_last_executor_cmd.has_control_mode &&
                 s_last_executor_cmd.control_mode == CONTROL_MODE_OPEN,
             "set_ctrl_mode did not publish explicit mode through executor")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(iq, (uint16_t)strlen((const char *)iq));
  if (Expect(s_executor_count == 1U && s_last_executor_cmd.has_iq_ref &&
                 s_last_executor_cmd.iq_ref == 1.25f,
             "set_Iq did not publish through executor")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(id, (uint16_t)strlen((const char *)id));
  if (Expect(s_executor_count == 1U && s_last_executor_cmd.has_id_ref &&
                 s_last_executor_cmd.id_ref == -0.5f,
             "set_Id did not publish through executor")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(torque, (uint16_t)strlen((const char *)torque));
  if (Expect(s_executor_count == 1U && s_last_executor_cmd.has_torque_ref &&
                 s_last_executor_cmd.iq_ref == 2.5f,
             "set_torque did not publish through executor")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(velocity, (uint16_t)strlen((const char *)velocity));
  if (Expect(s_executor_count == 1U && s_last_executor_cmd.has_velocity_ref &&
                 s_last_executor_cmd.speed_ref == Protocol_TurnsToRadians(3.5f),
             "set_vel did not publish through executor")) {
    return 1;
  }

  ResetHarness();
  vofa_Receive(position, (uint16_t)strlen((const char *)position));
  return Expect(s_executor_count == 1U &&
                    s_last_executor_cmd.has_position_ref &&
                    s_last_executor_cmd.position_ref ==
                        Protocol_TurnsToRadians(4.5f),
                "set_pos did not publish through executor");
}

static int TestClearFaultReportsRejectedState(void) {
  uint8_t command[] = "clear_fault=1";
  ResetHarness();
  s_motor_clear_result = false;

  vofa_Receive(command, (uint16_t)strlen((const char *)command));

  if (Expect(strstr(s_wire_log, "fault_clear=unsafe\n") != NULL,
             "clear fault did not report a rejected state")) {
    return 1;
  }
  return Expect(strstr(s_wire_log, "fault_clear=all\n") == NULL,
                "clear fault reported success after rejection");
}

static int TestSaveFlashReportsQueuedThenSucceeded(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  if (Expect(s_save_schedule_count == 1U,
             "save_flash did not schedule a flash save")) {
    return 1;
  }
  if (Expect(strstr(s_wire_log, "ack=save_flash,queued\n") != NULL,
             "save_flash did not emit queued acknowledgement")) {
    return 1;
  }
  CompleteTx();
  Vofa_ReportScheduledSaveResult(true);
  return Expect(strstr(s_wire_log, "ack=save_flash,succeeded\n") != NULL,
                "save_flash did not emit succeeded acknowledgement");
}

static int TestSaveFlashArmsBeforeSynchronousSuccess(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();
  s_save_commit_reports_success = true;

  vofa_Receive(command, (uint16_t)strlen((const char *)command));

  if (Expect(s_save_begin_count == 1U && s_save_commit_count == 1U,
             "save_flash did not use the begin/commit transaction")) {
    return 1;
  }
  if (Expect(strstr(s_wire_log, "ack=save_flash,queued\n") != NULL,
             "save_flash did not emit queued acknowledgement before commit")) {
    return 1;
  }
  CompleteTx();
  return Expect(strstr(s_wire_log, "ack=save_flash,succeeded\n") != NULL,
                "save_flash lost a synchronous success report from commit");
}

static int TestSaveFlashRetriesTerminalAckWhenQueueIsFull(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();
  s_save_commit_reports_success = true;

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH - 1U; i++) {
    char fill[16];
    snprintf(fill, sizeof(fill), "fill%u", i);
    if (Expect(Studio_SendText(fill), "failed to prefill VOFA TX queue")) {
      return 1;
    }
  }

  vofa_Receive(command, (uint16_t)strlen((const char *)command));

  if (Expect(s_save_commit_count == 1U, "save_flash did not commit")) {
    return 1;
  }
  if (Expect(strstr(s_wire_log, "ack=save_flash,succeeded\n") == NULL,
             "terminal ACK unexpectedly queued before TX space existed")) {
    return 1;
  }

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH + 2U; i++) {
    CompleteTx();
    Vofa_Service();
  }

  return Expect(CountWireOccurrences("ack=save_flash,succeeded\n") == 1U,
                "save_flash terminal ACK was not retried exactly once");
}

static int TestSaveFlashRejectsSecondRequestWhileTerminalAckPending(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();
  s_save_commit_reports_success = true;

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH - 1U; i++) {
    char fill[16];
    snprintf(fill, sizeof(fill), "fill%u", i);
    if (Expect(Studio_SendText(fill), "failed to prefill VOFA TX queue")) {
      return 1;
    }
  }

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  if (Expect(s_save_begin_count == 1U && s_save_commit_count == 1U,
             "first save_flash did not commit")) {
    return 1;
  }
  if (Expect(s_save_flash_result_pending &&
                 s_save_flash_terminal_ack_pending ==
                     SAVE_FLASH_TERMINAL_ACK_SUCCEEDED,
             "first save_flash terminal ACK was not pending after full queue")) {
    return 1;
  }

  if (Expect(Vofa_QueueReceive(command, (uint16_t)strlen((const char *)command)),
             "failed to queue second save_flash through RX queue")) {
    return 1;
  }

  CompleteTx();
  Vofa_Service();

  if (Expect(s_save_begin_count == 1U && s_save_commit_count == 1U,
             "second save_flash acquired a lease while terminal ACK pending")) {
    return 1;
  }
  if (Expect(s_save_flash_result_pending &&
                 s_save_flash_terminal_ack_pending ==
                     SAVE_FLASH_TERMINAL_ACK_SUCCEEDED,
             "second save_flash cleared the first terminal ACK")) {
    return 1;
  }

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH + 4U; i++) {
    CompleteTx();
    Vofa_Service();
  }

  if (Expect(CountWireOccurrences("ack=save_flash,busy\n") == 1U,
             "second save_flash did not report busy exactly once")) {
    return 1;
  }
  return Expect(CountWireOccurrences("ack=save_flash,succeeded\n") == 1U,
                "first save_flash terminal ACK was not eventually delivered exactly once");
}

static int TestSaveFlashTerminalAckClaimPreventsDuplicateSender(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  CompleteTx();

  CRITICAL_SECTION_BEGIN();
  s_save_flash_terminal_ack_pending = SAVE_FLASH_TERMINAL_ACK_SUCCEEDED;
  s_save_flash_terminal_ack_sending = true;
  CRITICAL_SECTION_END();

  Vofa_Service();
  if (Expect(CountWireOccurrences("ack=save_flash,succeeded\n") == 0U,
             "save_flash terminal ACK duplicated while another sender held claim")) {
    return 1;
  }

  CRITICAL_SECTION_BEGIN();
  s_save_flash_terminal_ack_sending = false;
  CRITICAL_SECTION_END();

  Vofa_Service();
  return Expect(CountWireOccurrences("ack=save_flash,succeeded\n") == 1U,
                "save_flash terminal ACK was not sent after claim release");
}

static int TestSaveFlashCancelsWhenQueuedAckCannotBeSent(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH; i++) {
    char fill[16];
    snprintf(fill, sizeof(fill), "fill%u", i);
    if (Expect(Studio_SendText(fill), "failed to fill VOFA TX queue")) {
      return 1;
    }
  }

  vofa_Receive(command, (uint16_t)strlen((const char *)command));

  if (Expect(s_save_begin_count == 1U, "save_flash did not acquire lease")) {
    return 1;
  }
  if (Expect(s_save_cancel_count == 1U,
             "save_flash did not cancel the lease when ACK queueing failed")) {
    return 1;
  }
  if (Expect(s_save_commit_count == 0U,
             "save_flash committed after ACK queueing failed")) {
    return 1;
  }
  Vofa_ReportScheduledSaveResult(true);
  return Expect(strstr(s_wire_log, "ack=save_flash,succeeded\n") == NULL,
                "save_flash reported success after the failed ACK was cancelled");
}

static int TestSaveFlashRetriesTerminalFailedAckWhenQueueIsFull(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH - 1U; i++) {
    char fill[16];
    snprintf(fill, sizeof(fill), "fill%u", i);
    if (Expect(Studio_SendText(fill), "failed to prefill VOFA TX queue")) {
      return 1;
    }
  }

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  Vofa_ReportScheduledSaveFailed();

  if (Expect(strstr(s_wire_log, "ack=save_flash,failed\n") == NULL,
             "failed terminal ACK unexpectedly queued before TX space existed")) {
    return 1;
  }

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH + 2U; i++) {
    CompleteTx();
    Vofa_Service();
  }

  return Expect(CountWireOccurrences("ack=save_flash,failed\n") == 1U,
                "failed terminal ACK was not retried exactly once");
}

static int TestSaveFlashReportsFailed(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  CompleteTx();
  Vofa_ReportScheduledSaveResult(false);
  if (Expect(strstr(s_wire_log, "ack=save_flash,retrying\n") != NULL,
             "save_flash did not emit retrying acknowledgement")) {
    return 1;
  }
  CompleteTx();
  Vofa_ReportScheduledSaveResult(true);
  return Expect(strstr(s_wire_log, "ack=save_flash,succeeded\n") != NULL,
                "save_flash did not acknowledge a later successful retry");
}

static int TestSaveFlashBusyWhileOperationEnabled(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();
  g_ds402_state_machine.current_state = STATE_OPERATION_ENABLED;

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  if (Expect(s_save_schedule_count == 0U,
             "operation-enabled save_flash scheduled a flash save")) {
    return 1;
  }
  return Expect(strstr(s_wire_log, "ack=save_flash,busy\n") != NULL,
                "operation-enabled save_flash did not report busy");
}

static int TestSaveFlashBusyWhileCalibrating(void) {
  uint8_t command[] = "save_flash=1";
  ResetHarness();
  g_ds402_state_machine.current_state = STATE_CALIBRATING;

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  if (Expect(s_save_schedule_count == 0U,
             "calibrating save_flash scheduled a flash save")) {
    return 1;
  }
  return Expect(strstr(s_wire_log, "ack=save_flash,busy\n") != NULL,
                "calibrating save_flash did not report busy");
}

#if defined(BOARD_XSTAR)
static int TestXstarBootEnterReportsUnsupported(void) {
  uint8_t command[] = "boot_enter";
  ResetHarness();

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  Vofa_Service();
  if (Expect(strstr(s_wire_log, "boot_ack,2,unsupported\n") != NULL,
             "boot_enter did not report unsupported")) {
    return 1;
  }
  if (Expect(!s_boot_upgrade_pending,
             "unsupported boot_enter armed a bootloader transition")) {
    return 1;
  }
  return Expect(s_boot_request_count == 0U,
                "unsupported boot_enter requested boot upgrade");
}
#endif

static int TestRxQueueOverflowIsObservable(void) {
  uint8_t command[] = "noop";
  ResetHarness();

  for (unsigned i = 0U; i < VOFA_RX_QUEUE_DEPTH; i++) {
    if (Expect(Vofa_QueueReceive(command,
                                 (uint16_t)strlen((const char *)command)),
               "failed to fill receive queue")) {
      return 1;
    }
  }
  if (Expect(!Vofa_QueueReceive(command,
                                (uint16_t)strlen((const char *)command)),
             "receive queue accepted an overflowing packet")) {
    return 1;
  }
  if (Expect(Vofa_GetReceiveOverflowCount() == 1U,
             "receive overflow counter did not increment")) {
    return 1;
  }
  Vofa_Service();
  return Expect(strstr(s_wire_log, "rx_overflow=1\n") != NULL,
                "receive overflow was not reported to the USB host");
}

static int TestShortcutUsesParamApi(const char *command, uint16_t expected) {
  uint8_t buf[64];
  ResetHarness();
  snprintf((char *)buf, sizeof(buf), "%s=3.25", command);
  vofa_Receive(buf, (uint16_t)strlen((const char *)buf));
  if (Expect(s_write_from_float_count == 1U,
             "shortcut command did not call Param_WriteFromFloat")) {
    return 1;
  }
  if (Expect(s_param_index == expected && s_param_value == 3.25f,
             "shortcut command used the wrong parameter index or value")) {
    return 1;
  }
  return 0;
}

static int TestCoggingCalibrationShortcutUsesParamApi(void) {
  uint8_t cmd[] = "set_cogging_calib=1";
  ResetHarness();
  vofa_Receive(cmd, (uint16_t)strlen((const char *)cmd));
  if (Expect(s_write_from_float_count == 1U,
             "set_cogging_calib did not call Param_WriteFromFloat")) {
    return 1;
  }
  return Expect(s_param_index == PARAM_COGGING_CALIB && s_param_value == 1.0f,
                "set_cogging_calib used the wrong parameter write");
}

static int TestGetParamUsesWireReadApi(void) {
  uint8_t cmd[] = "get_param=8192";
  ResetHarness();
  s_param_value = 4.75f;
  vofa_Receive(cmd, (uint16_t)strlen((const char *)cmd));
  if (Expect(s_read_as_float_count == 1U,
             "get_param did not call Param_ReadAsFloat")) {
    return 1;
  }
  if (Expect(s_param_index == PARAM_MOTOR_RS,
             "get_param passed the wrong index")) {
    return 1;
  }
  return Expect(strstr(s_wire_log, "param=8192,4.75") != NULL,
                "get_param did not send the API read value");
}

static int TestTxBufferIsStableUntilAsyncCompletion(void) {
  ResetHarness();
  if (Expect(Studio_SendText("first"), "first async transmit was rejected")) {
    return 1;
  }
  if (Expect(Studio_SendText("second"),
             "queued transmit while busy was rejected")) {
    return 1;
  }
  CompleteTx();
  CompleteTx();
  return Expect(strcmp(s_wire_log, "first\nsecond\n") == 0,
                "queued text was dropped, overwritten, or reordered");
}

static int TestTxBusyCanBeRetried(void) {
  ResetHarness();
  s_transmit_result = USBD_BUSY;
  if (Expect(Studio_SendText("retry"),
             "VOFA queue rejected a message during USB BUSY")) {
    return 1;
  }
  s_transmit_result = USBD_OK;
  Vofa_Service();
  CompleteTx();
  return Expect(strstr(s_wire_log, "retry") != NULL,
                "retried transmit was not submitted");
}

static int TestTxCompletionCanInterleaveWithoutDroppingOrder(void) {
  ResetHarness();
  if (!Studio_SendText("one") || !Studio_SendText("two") ||
      !Studio_SendText("three")) {
    printf("FAIL queue setup rejected an item\n");
    return 1;
  }
  CompleteTx();
  if (Expect(Studio_SendText("four"),
             "transmit during completion drain was rejected")) {
    return 1;
  }
  CompleteTx();
  CompleteTx();
  CompleteTx();
  return Expect(strcmp(s_wire_log, "one\ntwo\nthree\nfour\n") == 0,
                "completion interleaving dropped or reordered TX items");
}

static int TestBootEnterWaitsForAckCompletion(void) {
  uint8_t command[] = "boot_enter";
  ResetHarness();

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  if (Expect(strstr(s_wire_log, "boot_ack,0,entering_bootloader\n") != NULL,
             "boot_enter did not queue its acknowledgement")) {
    return 1;
  }
  if (Expect(s_delay_call_count == 0U,
             "boot_enter blocked the USB receive callback")) {
    return 1;
  }

  Vofa_Service();
  if (Expect(s_boot_request_count == 0U,
             "boot upgrade was requested before ACK completion")) {
    return 1;
  }

  CompleteTx();
  if (Expect(s_boot_request_count == 0U,
             "USB completion callback requested the boot upgrade")) {
    return 1;
  }
  Vofa_Service();
  if (Expect(s_boot_request_count == 1U,
             "boot upgrade was not requested after ACK completion")) {
    return 1;
  }
  Vofa_Service();
  return Expect(s_boot_request_count == 1U,
                "boot upgrade request was issued more than once");
}

static int TestBootEnterDoesNotRebootWhenAckQueueIsFull(void) {
  uint8_t command[] = "boot_enter";
  ResetHarness();

  for (unsigned i = 0U; i < VOFA_TX_QUEUE_DEPTH; i++) {
    if (Expect(Studio_SendText("queued"),
               "failed to fill the VOFA transmit queue")) {
      return 1;
    }
  }

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  Vofa_Service();
  if (Expect(!s_boot_upgrade_pending,
             "boot upgrade was armed without a queued acknowledgement")) {
    return 1;
  }
  return Expect(s_boot_request_count == 0U,
                "boot upgrade was requested after ACK queue rejection");
}

static int TestCommandNamesRequireTokenBoundary(void) {
  uint8_t boot_command[] = "boot_enter_now";
  uint8_t version_command[] = "get_version_extra";
  ResetHarness();

  vofa_Receive(boot_command, (uint16_t)strlen((const char *)boot_command));
  vofa_Receive(version_command,
               (uint16_t)strlen((const char *)version_command));
  Vofa_Service();

  if (Expect(s_tx_count == 0U,
             "a command name with a trailing suffix was accepted")) {
    return 1;
  }
  if (Expect(!s_boot_upgrade_pending && s_boot_request_count == 0U,
             "a suffixed boot command armed a bootloader transition")) {
    return 1;
  }
  return Expect(s_wire_log[0] == '\0',
                "a suffixed command produced a response");
}

static int TestReadAllParamsResumesAfterQueueBackpressure(void) {
  uint8_t command[] = "read_all_params";
  ResetHarness();
  s_param_value = 2.5f;
  s_sync_entry_count = 10U;
  for (uint32_t i = 0U; i < s_sync_entry_count; i++) {
    s_sync_entries[i].index = (uint16_t)(0x2100U + i);
  }

  vofa_Receive(command, (uint16_t)strlen((const char *)command));
  unsigned guard = 0U;
  while ((s_param_sync_active || s_tx_count > 0U || s_hcdc.TxState != 0U) &&
         guard++ < 100U) {
    Vofa_Service();
    if (s_hcdc.TxState != 0U) {
      CompleteTx();
    }
  }

  if (Expect(guard < 100U, "parameter sync did not drain")) {
    return 1;
  }
  unsigned response_count = 0U;
  const char *response = s_wire_log;
  while ((response = strstr(response, "param=")) != NULL) {
    response_count++;
    response += strlen("param=");
  }
  if (Expect(response_count == s_sync_entry_count,
             "parameter sync skipped or duplicated a response")) {
    return 1;
  }
  for (uint32_t i = 0U; i < s_sync_entry_count; i++) {
    char expected[24];
    snprintf(expected, sizeof(expected), "param=%u,",
             (unsigned)s_sync_entries[i].index);
    if (Expect(strstr(s_wire_log, expected) != NULL,
               "parameter sync omitted an indexed response")) {
      return 1;
    }
  }
  return Expect(strstr(s_wire_log, "param_sync_done\n") != NULL,
                "parameter sync did not emit completion marker");
}

int main(void) {
  if (TestUsbReceiveDefersCommandToTaskService())
    return 1;
  if (TestSetParamUsesWireWriteApi())
    return 1;
  if (TestSetParamRejectsWireWriteFailure())
    return 1;
  if (TestSetParamStrictDecimalParsing())
    return 1;
  if (TestMalformedCommandsDoNotMutateControlState())
    return 1;
  if (TestVofaMotorCommandsUseExecutor())
    return 1;
  if (TestClearFaultReportsRejectedState())
    return 1;
  if (TestSaveFlashReportsQueuedThenSucceeded())
    return 1;
  if (TestSaveFlashArmsBeforeSynchronousSuccess())
    return 1;
  if (TestSaveFlashRetriesTerminalAckWhenQueueIsFull())
    return 1;
  if (TestSaveFlashRejectsSecondRequestWhileTerminalAckPending())
    return 1;
  if (TestSaveFlashTerminalAckClaimPreventsDuplicateSender())
    return 1;
  if (TestSaveFlashCancelsWhenQueuedAckCannotBeSent())
    return 1;
  if (TestSaveFlashRetriesTerminalFailedAckWhenQueueIsFull())
    return 1;
  if (TestSaveFlashReportsFailed())
    return 1;
  if (TestSaveFlashBusyWhileOperationEnabled())
    return 1;
  if (TestSaveFlashBusyWhileCalibrating())
    return 1;
#if defined(BOARD_XSTAR)
  if (TestXstarBootEnterReportsUnsupported())
    return 1;
#endif
  if (TestRxQueueOverflowIsObservable())
    return 1;
  if (TestShortcutUsesParamApi("set_vel_kp", PARAM_SPD_KP))
    return 1;
  if (TestShortcutUsesParamApi("set_vel_ki", PARAM_SPD_KI))
    return 1;
  if (TestShortcutUsesParamApi("set_pos_kp", PARAM_POS_KP))
    return 1;
  if (TestShortcutUsesParamApi("set_ladrc_en", PARAM_LADRC_ENABLE))
    return 1;
  if (TestShortcutUsesParamApi("set_ladrc_wo", PARAM_LADRC_OMEGA_O))
    return 1;
  if (TestShortcutUsesParamApi("set_ladrc_wc", PARAM_LADRC_OMEGA_C))
    return 1;
  if (TestShortcutUsesParamApi("set_ladrc_b0", PARAM_LADRC_B0))
    return 1;
  if (TestShortcutUsesParamApi("set_cogging_enable", PARAM_COGGING_EN))
    return 1;
  if (TestCoggingCalibrationShortcutUsesParamApi())
    return 1;
  if (TestGetParamUsesWireReadApi())
    return 1;
  if (TestTxBufferIsStableUntilAsyncCompletion())
    return 1;
  if (TestTxBusyCanBeRetried())
    return 1;
  if (TestTxCompletionCanInterleaveWithoutDroppingOrder())
    return 1;
#if !defined(BOARD_XSTAR)
  if (TestBootEnterWaitsForAckCompletion())
    return 1;
  if (TestBootEnterDoesNotRebootWhenAckQueueIsFull())
    return 1;
#endif
  if (TestCommandNamesRequireTokenBoundary())
    return 1;
  if (TestReadAllParamsResumesAfterQueueBackpressure())
    return 1;
  printf("VOFA command and USB TX queue regression tests passed\n");
  return 0;
}
