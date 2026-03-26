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
#include "config.h"
#include "control/cogging.h"
#include "control/control.h"
#include "control/ladrc.h"
#include "fault_def.h"
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "safety_control.h"
#include "usbd_cdc_if.h"
#include "version.h"
#include "bootloader.h"
#include <stdarg.h>
#include <stdio.h>
/* ============================================================================
 *
 * ============================================================================
 */
#define MAX_TXBUFFER_SIZE 512
#define MAX_RXBUFFER_SIZE 512
#define TEXT_LINE_MAX 256
static uint8_t send_buf[MAX_TXBUFFER_SIZE];
static uint8_t receive_buf[MAX_RXBUFFER_SIZE];
static uint16_t cnt = 0;
/* ============================================================================
 *
 * ============================================================================
 */
#define USART_OR_CDC 1
static void vofa_transmit(const uint8_t *buf, uint16_t len) {
  if (len > 0 && len <= MAX_TXBUFFER_SIZE) {
#if USART_OR_CDC == 0
    static uint8_t dma_buf[MAX_TXBUFFER_SIZE];
    memcpy(dma_buf, buf, len);
    HAL_UART_Transmit_DMA(&huart3, dma_buf, len);
#elif USART_OR_CDC == 1
    CDC_Transmit_FS((uint8_t *)buf, len);
#endif
  }
}
/* ============================================================================
 *   API
 * ============================================================================
 */
void Studio_SendText(const char *text) {
  if (text == NULL)
    return;
  uint16_t len = (uint16_t)strlen(text);
  //  '\n',
  if (len + 1 > MAX_TXBUFFER_SIZE)
    return;
  static uint8_t text_buf[TEXT_LINE_MAX + 2];
  memcpy(text_buf, text, len);
  text_buf[len] = '\n';
  vofa_transmit(text_buf, len + 1);
}
void Studio_SendTextf(const char *fmt, ...) {
  static char fmt_buf[TEXT_LINE_MAX];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(fmt_buf, sizeof(fmt_buf), fmt, args);
  va_end(args);
  if (n > 0) {
    Studio_SendText(fmt_buf);
  }
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
  vofa_send_data(7, ENC(&motor_data)->vel_estimate_);
  vofa_send_data(8, ENC(&motor_data)->pos_estimate_);
  vofa_send_data(9, ENC(&motor_data)->phase_);
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
  if (ENC(&motor_data)) {
    data[7] = ENC(&motor_data)->vel_estimate_;
    data[8] = ENC(&motor_data)->pos_estimate_;
    data[9] = ENC(&motor_data)->phase_;
  } else {
    data[7] = 0;
    data[8] = 0;
    data[9] = 0;
  }
  data[10] = motor_data.feedback.temperature;
  scope_buf.head = next_head;
}
void Scope_Process(void) {
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
    {FAULT_CURRENT_A, "OC_A", "CRITICAL"},
};
#define FAULT_MAP_SIZE (sizeof(s_fault_map) / sizeof(FaultMap_t))

bool Studio_IsScopeEnabled(void) { return s_status.scope_enabled; }
void Studio_ReportVersion(void) {
  Studio_SendTextf("fw_version=%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
                   FW_VERSION_PATCH);
}
void Studio_ReportCalibStatus(void) {
  bool calibrating =
      (motor_data.state.State_Mode == STATE_MODE_DETECTING) &&
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
      "state=mode:%d,en:%d,vel:%.2f,pos:%.3f,tq:%.3f,iq:%.3f,ladrc:%d",
      mode, enabled, motor_data.Controller.input_velocity,
      motor_data.Controller.input_position,
      motor_data.Controller.input_torque, motor_data.algo_output.Iq, ladrc);
}
void Studio_PeriodicUpdate(void) {
  /*  */
  if (!s_status.version_sent) {
    Studio_ReportVersion();
    s_status.version_sent = true;
  }
  /* ── calibrationstate ── */
  bool calibrating =
      (motor_data.state.State_Mode == STATE_MODE_DETECTING) &&
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
    if (step != s_status.last_cogging_step &&
        (step % 10 == 0 || step == 0)) {
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
static float vofa_cmd_parse(const char *recvStr, const char *arg) {
  const char *pos = strstr(recvStr, arg);
  if (pos == NULL)
    return 0.0f;
  return atof(pos + strlen(arg));
}
static int vofa_cmd_parse_int(const char *recvStr, const char *arg) {
  const char *pos = strstr(recvStr, arg);
  if (pos == NULL)
    return 0;
  return atoi(pos + strlen(arg));
}
/* ============================================================================
 *   (VectorStudio → )
 * ============================================================================
 */
void vofa_Receive(uint8_t *buf, uint16_t len) {
  if (len >= MAX_RXBUFFER_SIZE)
    len = MAX_RXBUFFER_SIZE - 1;
  memcpy(receive_buf, buf, len);
  receive_buf[len] = '\0';
  char *recvStr = (char *)receive_buf;
  /* ──  ── */
  if (strstr(recvStr, "handshake=studio")) {
    Studio_SendTextf("ack=studio,%d.%d.%d", FW_VERSION_MAJOR,
                     FW_VERSION_MINOR, FW_VERSION_PATCH);
    s_status.version_sent = true;
    return;
  }
  if (strstr(recvStr, "get_version")) {
    Studio_ReportVersion();
    return;
  }
  if (strstr(recvStr, "get_param=")) {
    int idx = vofa_cmd_parse_int(recvStr, "get_param=");
    const ParamEntry *entry = ParamTable_Find((uint16_t)idx);
    if (entry && entry->type == PARAM_TYPE_FLOAT) {
      Studio_SendTextf("param=%d,%.6f", idx, *(float *)entry->ptr);
    } else if (entry && entry->type == PARAM_TYPE_UINT8) {
      Studio_SendTextf("param=%d,%d", idx, (int)*(uint8_t *)entry->ptr);
    } else if (entry && entry->type == PARAM_TYPE_UINT32) {
      Studio_SendTextf("param=%d,%u", idx, (unsigned)*(uint32_t *)entry->ptr);
    } else {
      Studio_SendTextf("param=%d,ERR", idx);
    }
    return;
  }
  if (strstr(recvStr, "set_param=")) {
    // param: set_param=INDEX,VALUE
    const char *payload = strstr(recvStr, "set_param=") + 10;
    int idx = 0;
    float val = 0;
    if (sscanf(payload, "%d,%f", &idx, &val) == 2) {
      const ParamEntry *entry = ParamTable_Find((uint16_t)idx);
      if (entry == NULL) {
        Studio_SendTextf("param_err=%d,NOT_FOUND", idx);
      } else if (!(entry->access & PARAM_ACCESS_W)) {
        Studio_SendTextf("param_err=%d,READONLY", idx);
      } else if (val < entry->min || val > entry->max) {
        Studio_SendTextf("param_err=%d,OUT_OF_RANGE", idx);
      } else {
        switch (entry->type) {
        case PARAM_TYPE_FLOAT:
          *(float *)entry->ptr = val;
          break;
        case PARAM_TYPE_UINT8:
          *(uint8_t *)entry->ptr = (uint8_t)val;
          break;
        case PARAM_TYPE_UINT16:
          *(uint16_t *)entry->ptr = (uint16_t)val;
          break;
        case PARAM_TYPE_UINT32:
          *(uint32_t *)entry->ptr = (uint32_t)val;
          break;
        case PARAM_TYPE_INT32:
          *(int32_t *)entry->ptr = (int32_t)val;
          break;
        }
        motor_data.params_updated = true;
        Studio_SendTextf("param=%d,%.6g", idx, val);
      }
    }
    return;
  }
  if (strstr(recvStr, "read_all_params")) {
    // param
    uint32_t count = ParamTable_GetCount();
    const ParamEntry *table = ParamTable_GetTable();
    for (uint32_t i = 0; i < count; i++) {
      const ParamEntry *e = &table[i];
      switch (e->type) {
      case PARAM_TYPE_FLOAT:
        Studio_SendTextf("param=%d,%.6g", e->index, *(float *)e->ptr);
        break;
      case PARAM_TYPE_UINT8:
        Studio_SendTextf("param=%d,%d", e->index, (int)*(uint8_t *)e->ptr);
        break;
      case PARAM_TYPE_UINT16:
        Studio_SendTextf("param=%d,%d", e->index, (int)*(uint16_t *)e->ptr);
        break;
      case PARAM_TYPE_UINT32:
        Studio_SendTextf("param=%d,%u", e->index,
                         (unsigned)*(uint32_t *)e->ptr);
        break;
      case PARAM_TYPE_INT32:
        Studio_SendTextf("param=%d,%d", e->index, (int)*(int32_t *)e->ptr);
        break;
      }
    }
    Studio_SendText("param_sync_done");
    return;
  }
  if (strstr(recvStr, "save_flash=1")) {
    Param_ScheduleSave();
    Studio_SendText("ack=save_ok");
    return;
  }
  /* ── motor ── */
  if (strstr(recvStr, "motor_enable=")) {
    float motor_enable = vofa_cmd_parse(recvStr, "motor_enable=");
    if (motor_enable > 0.5f) {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_OPERATION_ENABLED);
    } else {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
    return;
  }
  if (strstr(recvStr, "calib=")) {
    float calib_enable = vofa_cmd_parse(recvStr, "calib=");
    if (calib_enable > 0.5f) {
      s_status.last_calib_active = false; //
      s_status.last_calib_step = -1;
      StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
      Studio_SendText("calib_step=0");
    }
    return;
  }
  if (strstr(recvStr, "clear_fault=1")) {
    Motor_ClearFaults(&motor_data);
    s_status.last_fault_bits = 0;
    Studio_SendText("fault_clear=all");
    return;
  }
  if (strstr(recvStr, "set_zero=1")) {
    // setposition
    if (ENC(&motor_data)) {
      ENC(&motor_data)->offset_counts = ENC(&motor_data)->raw_angle;
      ENC(&motor_data)->pos_estimate_ = 0.0f;
      Param_ScheduleSave();
      Studio_SendText("ack=zero_set");
    }
    return;
  }
  if (strstr(recvStr, "set_ctrl_mode=")) {
    int ctrlModeInt = vofa_cmd_parse_int(recvStr, "set_ctrl_mode=");
    if (ctrlModeInt >= CONTROL_MODE_OPEN &&
        ctrlModeInt <= CONTROL_MODE_MIT) {
      motor_data.state.Control_Mode = (CONTROL_MODE)ctrlModeInt;
    }
    return;
  }
  /* ──  ── */
  if (strstr(recvStr, "set_Iq=")) {
    motor_data.algo_input.Iq_ref = vofa_cmd_parse(recvStr, "set_Iq=");
    return;
  }
  if (strstr(recvStr, "set_Id=")) {
    motor_data.algo_input.Id_ref = vofa_cmd_parse(recvStr, "set_Id=");
    return;
  }
  if (strstr(recvStr, "set_torque=")) {
    motor_data.Controller.input_torque =
        vofa_cmd_parse(recvStr, "set_torque=");
    return;
  }
  if (strstr(recvStr, "set_vel=")) {
    motor_data.Controller.input_velocity =
        vofa_cmd_parse(recvStr, "set_vel=");
    return;
  }
  if (strstr(recvStr, "set_pos=")) {
    motor_data.Controller.input_position =
        vofa_cmd_parse(recvStr, "set_pos=");
    motor_data.Controller.input_updated = true;
    return;
  }
  /* ── PID  ── */
  if (strstr(recvStr, "set_current_ctrl_bw=")) {
    motor_data.Controller.current_ctrl_bandwidth =
        vofa_cmd_parse(recvStr, "set_current_ctrl_bw=");
    CurrentLoop_UpdateGain(&motor_data);
    motor_data.params_updated = true;
    return;
  }
  if (strstr(recvStr, "set_vel_kp=")) {
    motor_data.VelPID.Kp = vofa_cmd_parse(recvStr, "set_vel_kp=");
    return;
  }
  if (strstr(recvStr, "set_vel_ki=")) {
    motor_data.VelPID.Ki = vofa_cmd_parse(recvStr, "set_vel_ki=");
    return;
  }
  if (strstr(recvStr, "set_pos_kp=")) {
    motor_data.PosPID.Kp = vofa_cmd_parse(recvStr, "set_pos_kp=");
    return;
  }
  /* ── LADRC speed/velocityparam ── */
  if (strstr(recvStr, "set_ladrc_en=")) {
    motor_data.ladrc_enable = vofa_cmd_parse(recvStr, "set_ladrc_en=");
    // reset LADRC  PID state
    if (motor_data.ladrc_enable >= 0.5f) {
      LADRC_Reset(&motor_data.ladrc_state);
      LADRC_Init(&motor_data.ladrc_state, &motor_data.ladrc_config);
    } else {
      PID_clear(&motor_data.VelPID);
    }
    return;
  }
  if (strstr(recvStr, "set_ladrc_wo=")) {
    motor_data.ladrc_config.omega_o =
        vofa_cmd_parse(recvStr, "set_ladrc_wo=");
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    return;
  }
  if (strstr(recvStr, "set_ladrc_wc=")) {
    motor_data.ladrc_config.omega_c =
        vofa_cmd_parse(recvStr, "set_ladrc_wc=");
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    return;
  }
  if (strstr(recvStr, "set_ladrc_b0=")) {
    motor_data.ladrc_config.b0 = vofa_cmd_parse(recvStr, "set_ladrc_b0=");
    return;
  }
  /* ──  ── */
  if (strstr(recvStr, "set_cogging_calib=1")) {
    motor_data.advanced.cogging_calib_request = 1.0f;
    Studio_SendText("ack=cogging_calib_started");
    return;
  }
  if (strstr(recvStr, "set_cogging_enable=")) {
    motor_data.advanced.cogging_comp_enabled =
        vofa_cmd_parse(recvStr, "set_cogging_enable=");
    return;
  }
  /* ──  ── */
  // setstatefrequency: set_status_rate=N (0=, 1=10Hz, 2=5Hz, 5=2Hz, 10=1Hz)
  if (strstr(recvStr, "set_status_rate=")) {
    int rate = vofa_cmd_parse_int(recvStr, "set_status_rate=");
    s_status.status_rate_div = (uint8_t)CLAMP(rate, 0, 100);
    s_status.status_tick = 0;
    Studio_SendTextf("ack=status_rate,%d", (int)s_status.status_rate_div);
    return;
  }
  //  (scope) enable/: set_scope_enable=0/1
  if (strstr(recvStr, "set_scope_enable=")) {
    float en = vofa_cmd_parse(recvStr, "set_scope_enable=");
    s_status.scope_enabled = (en > 0.5f);
    Studio_SendTextf("ack=scope_enable,%d", s_status.scope_enabled ? 1 : 0);
    return;
  }
  // state ()
  if (strstr(recvStr, "get_status")) {
    Studio_ReportMotorStatus();
    return;
  }
  /* ── CAN  () ── */
  if (strstr(recvStr, "scan_bus=1")) {
    Studio_SendText("bus_scan_start");
    // TODO: actual， CAN ID
    //
    Studio_SendTextf("bus_node=%d,online,%.0f", g_can_id,
                     motor_data.feedback.temperature);
    Studio_SendText("bus_scan_done");
    return;
  }
  /* ── OTA 升级命令 ── */
  if (strstr(recvStr, "boot_enter")) {
    /* 先禁用电机 */
    StateMachine_RequestState(&g_ds402_state_machine, STATE_SWITCH_ON_DISABLED);
    /* 发送确认消息 */
    Studio_SendText("boot_ack,0,entering_bootloader");
    /* 等待消息发送完成 */
    HAL_Delay(100);
    /* 请求进入 Bootloader */
    Boot_RequestUpgrade();
    /* 不会返回 */
    return;
  }
}
