#include "vofa.h"
#include "bootloader.h"
#include "config.h"
#include "control/cogging.h"
#include "control/control.h"
#include "control/ladrc.h"
#include "device_id.h"
#include "fault_def.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"
#include "inovxio_protocol.h"
#include "manager.h"
#include "motor.h"
#include "param_access.h"
#include "param_table.h"
#include "pid.h"
#include "safety_control.h"
#include "usbd_cdc_if.h"
#include "version.h"
#include "vofa_bus_scan.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TXBUFFER_SIZE 512
#define MAX_RXBUFFER_SIZE 512
#define TEXT_LINE_MAX 256
#define VOFA_SCAN_TIMEOUT_MS 120U
#define VOFA_SCAN_BROADCAST_ID 0x7FU

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

static uint8_t send_buf[MAX_TXBUFFER_SIZE];
static uint8_t receive_buf[MAX_RXBUFFER_SIZE];
static uint16_t cnt = 0;
static ScopeBuffer_t scope_buf;
static VofaBusScanState s_bus_scan;
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

static void vofa_transmit(const uint8_t *buf, uint16_t len) {
  if (len == 0U || len > MAX_TXBUFFER_SIZE) {
    return;
  }

  CDC_Transmit_FS((uint8_t *)buf, len);
}

static void Studio_SendText(const char *text) {
  static uint8_t text_buf[TEXT_LINE_MAX + 2];
  size_t len = 0;

  if (text == NULL) {
    return;
  }

  len = strlen(text);
  if (len > TEXT_LINE_MAX) {
    len = TEXT_LINE_MAX;
  }

  memcpy(text_buf, text, len);
  text_buf[len++] = '\n';
  vofa_transmit(text_buf, (uint16_t)len);
}

static void Studio_SendTextf(const char *fmt, ...) {
  char text_buf[TEXT_LINE_MAX + 1];
  va_list args;
  int written = 0;

  va_start(args, fmt);
  written = vsnprintf(text_buf, sizeof(text_buf), fmt, args);
  va_end(args);

  if (written <= 0) {
    return;
  }

  Studio_SendText(text_buf);
}

static float vofa_cmd_parse(const char *recv_str, const char *arg) {
  const char *pos = strstr(recv_str, arg);

  if (pos == NULL) {
    return 0.0f;
  }

  return atof(pos + strlen(arg));
}

static int vofa_cmd_parse_int(const char *recv_str, const char *arg) {
  const char *pos = strstr(recv_str, arg);

  if (pos == NULL) {
    return 0;
  }

  return atoi(pos + strlen(arg));
}

static int Studio_CsStateToStep(CS_STATE cs) {
  if (cs >= CS_MOTOR_R_START && cs <= CS_MOTOR_R_END) {
    return 1;
  }
  if (cs >= CS_MOTOR_L_START && cs <= CS_MOTOR_L_END) {
    return 2;
  }
  if (cs >= CS_DIR_PP_START && cs <= CS_DIR_PP_END) {
    return 3;
  }
  if (cs >= CS_ENCODER_START && cs <= CS_ENCODER_END) {
    return 4;
  }
  if (cs >= CS_FLUX_START && cs <= CS_FLUX_END) {
    return 5;
  }
  if (cs == CS_REPORT_OFFSET_LUT) {
    return 6;
  }
  return 0;
}

static void Studio_ReportVersion(void) {
  Studio_SendTextf("fw_version=%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
                   FW_VERSION_PATCH);
}

static void Studio_ReportMotorStatus(void) {
  MotorState ds402 = StateMachine_GetState(&g_ds402_state_machine);
  int enabled = (ds402 == STATE_OPERATION_ENABLED) ? 1 : 0;
  int ladrc = (motor_data.ladrc_enable >= 0.5f) ? 1 : 0;

  Studio_SendTextf(
      "state=mode:%d,en:%d,vel:%.2f,pos:%.3f,tq:%.3f,iq:%.3f,ladrc:%d",
      (int)motor_data.state.Control_Mode, enabled, motor_data.feedback.velocity,
      motor_data.feedback.position, motor_data.Controller.input_torque,
      motor_data.algo_output.Iq, ladrc);
}

static void Studio_HandleRunModeUpdate(uint8_t proto_mode) {
  CONTROL_MODE new_mode = CONTROL_MODE_OPEN;
  bool valid = true;

  switch (proto_mode) {
  case 0:
    new_mode = CONTROL_MODE_MIT;
    break;
  case 1:
    new_mode = CONTROL_MODE_POSITION_RAMP;
    break;
  case 2:
    new_mode = CONTROL_MODE_VELOCITY;
    break;
  case 3:
    new_mode = CONTROL_MODE_TORQUE;
    break;
  case 5:
    new_mode = CONTROL_MODE_POSITION;
    break;
  default:
    valid = false;
    break;
  }

  if (valid) {
    motor_data.state.Control_Mode = new_mode;
  }
}

static void Studio_HandleParamChange(uint16_t index) {
  switch (index) {
  case PARAM_RUN_MODE:
    Studio_HandleRunModeUpdate(g_run_mode);
    break;
  case PARAM_CUR_KP:
  case PARAM_CUR_KI:
  case PARAM_SPD_KP:
  case PARAM_SPD_KI:
  case PARAM_POS_KP:
    motor_data.params_updated = true;
    break;
  case PARAM_LADRC_ENABLE:
    if (motor_data.ladrc_enable >= 0.5f) {
      LADRC_Reset(&motor_data.ladrc_state);
      LADRC_Init(&motor_data.ladrc_state, &motor_data.ladrc_config);
    } else {
      PID_clear(&motor_data.VelPID);
    }
    break;
  case PARAM_LADRC_OMEGA_O:
  case PARAM_LADRC_OMEGA_C:
  case PARAM_LADRC_B0:
  case PARAM_LADRC_MAX_OUT:
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    break;
  case PARAM_PROTOCOL_TYPE:
    if (g_protocol_type <= PROTOCOL_MIT) {
      Protocol_SetType((ProtocolType)g_protocol_type);
    }
    break;
  default:
    break;
  }
}

static void Studio_WriteParam(uint16_t index, float value) {
  const ParamEntry *entry = ParamTable_Find(index);

  if (entry == NULL) {
    Studio_SendTextf("param_err=%u,NOT_FOUND", (unsigned)index);
    return;
  }

  if ((entry->access & PARAM_ACCESS_W) == 0U) {
    Studio_SendTextf("param_err=%u,READONLY", (unsigned)index);
    return;
  }

  if (value < entry->min || value > entry->max) {
    Studio_SendTextf("param_err=%u,OUT_OF_RANGE", (unsigned)index);
    return;
  }

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    *(float *)entry->ptr = value;
    break;
  case PARAM_TYPE_UINT8:
    *(uint8_t *)entry->ptr = (uint8_t)value;
    break;
  case PARAM_TYPE_UINT16:
    *(uint16_t *)entry->ptr = (uint16_t)value;
    break;
  case PARAM_TYPE_UINT32:
    *(uint32_t *)entry->ptr = (uint32_t)value;
    break;
  case PARAM_TYPE_INT32:
    *(int32_t *)entry->ptr = (int32_t)value;
    break;
  }

  motor_data.params_updated = true;
  Studio_HandleParamChange(index);
  Studio_SendTextf("param=%u,%.6g", (unsigned)index, value);
}

static void Studio_ReadParam(uint16_t index) {
  const ParamEntry *entry = ParamTable_Find(index);

  if (entry == NULL) {
    Studio_SendTextf("param=%u,ERR", (unsigned)index);
    return;
  }

  switch (entry->type) {
  case PARAM_TYPE_FLOAT:
    Studio_SendTextf("param=%u,%.6f", (unsigned)index, *(float *)entry->ptr);
    break;
  case PARAM_TYPE_UINT8:
    Studio_SendTextf("param=%u,%u", (unsigned)index,
                     (unsigned)*(uint8_t *)entry->ptr);
    break;
  case PARAM_TYPE_UINT16:
    Studio_SendTextf("param=%u,%u", (unsigned)index,
                     (unsigned)*(uint16_t *)entry->ptr);
    break;
  case PARAM_TYPE_UINT32:
    Studio_SendTextf("param=%u,%lu", (unsigned)index,
                     (unsigned long)*(uint32_t *)entry->ptr);
    break;
  case PARAM_TYPE_INT32:
    Studio_SendTextf("param=%u,%ld", (unsigned)index,
                     (long)*(int32_t *)entry->ptr);
    break;
  }
}

static void Studio_ReadAllParams(void) {
  const ParamEntry *table = ParamTable_GetTable();
  uint32_t count = ParamTable_GetCount();
  uint32_t i = 0;

  for (i = 0; i < count; ++i) {
    Studio_ReadParam(table[i].index);
  }

  Studio_SendText("param_sync_done");
}

static void Vofa_BusScanObserver(const CAN_Frame *frame) {
  VofaBusScan_ObserveFrame(&s_bus_scan, frame);
}

static bool Vofa_ProcessBusScan(void) {
  VofaBusScanEvent event = {0};

  if (!VofaBusScan_IsBusy(&s_bus_scan)) {
    return false;
  }

  if (VofaBusScan_PollEvent(&s_bus_scan, HAL_GetSystemTick(), &event)) {
    if (event.type == VOFA_BUS_SCAN_EVENT_NODE) {
      Studio_SendTextf("bus_node=%u,online,%08lX%08lX",
                       (unsigned)event.node.node_id,
                       (unsigned long)event.node.uid_word0,
                       (unsigned long)event.node.uid_word1);
    } else if (event.type == VOFA_BUS_SCAN_EVENT_DONE) {
      Studio_SendTextf("bus_scan_done,count=%u", (unsigned)event.total_nodes);
    }
  }

  return true;
}

static void Vofa_BeginBusScan(void) {
  CAN_Frame frame = {0};
  DeviceUID_t uid = {0};

  if (VofaBusScan_IsBusy(&s_bus_scan)) {
    return;
  }

  if (Protocol_GetType() != PROTOCOL_INOVXIO) {
    Studio_SendText("bus_scan_error=protocol");
    return;
  }

  DeviceID_GetUID(&uid);
  VofaBusScan_Begin(&s_bus_scan, HAL_GetSystemTick(), VOFA_SCAN_TIMEOUT_MS,
                    g_can_id, uid.word0, uid.word1);
  scope_buf.tail = scope_buf.head;
  Studio_SendText("bus_scan_start");

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | VOFA_SCAN_BROADCAST_ID;
  frame.dlc = 0;
  frame.is_extended = true;
  frame.is_rtr = false;

  if (!Protocol_SendFrame(&frame)) {
    Studio_SendText("bus_scan_warn=tx_failed");
  }
}

void vofa_start(void) { Vofa_Packet(); }

void vofa_send_data(uint8_t num, float data) {
  (void)num;

  if ((uint16_t)(cnt + 4U) < MAX_TXBUFFER_SIZE) {
    send_buf[cnt++] = byte0(data);
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
  }
}

void vofa_sendframetail(void) {
  if ((uint16_t)(cnt + 4U) < MAX_TXBUFFER_SIZE) {
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7F;
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

void Scope_Init(void) {
  scope_buf.head = 0;
  scope_buf.tail = 0;
  VofaBusScan_Init(&s_bus_scan);
  Protocol_RegisterRxObserver(Vofa_BusScanObserver);

  s_status.last_calib_step = -1;
  s_status.last_calib_active = false;
  s_status.last_fault_bits = 0;
  s_status.version_sent = false;
  s_status.cogging_was_active = false;
  s_status.last_cogging_step = 0;
  s_status.status_rate_div = 2;
  s_status.status_tick = 0;
  s_status.scope_enabled = true;
}

void Scope_Update(void) {
  uint16_t next_head = (uint16_t)((scope_buf.head + 1U) % SCOPE_BUFFER_SIZE);
  float *data = NULL;

  if (next_head == scope_buf.tail) {
    return;
  }

  data = scope_buf.data[scope_buf.head];
  data[0] = motor_data.algo_input.Ia;
  data[1] = motor_data.algo_input.Ib;
  data[2] = motor_data.algo_input.Ic;
  data[3] = motor_data.algo_output.Iq;
  data[4] = motor_data.algo_output.Id;
  data[5] = motor_data.algo_input.Iq_ref;
  data[6] = motor_data.algo_input.Id_ref;
  data[11] = motor_data.algo_input.Vbus;

  if (ENC(&motor_data) != NULL) {
    data[7] = ENC(&motor_data)->vel_estimate_;
    data[8] = ENC(&motor_data)->pos_estimate_;
    data[9] = ENC(&motor_data)->phase_;
  } else {
    data[7] = 0.0f;
    data[8] = 0.0f;
    data[9] = 0.0f;
  }

  data[10] = motor_data.feedback.temperature;
  scope_buf.head = next_head;
}

void Scope_Process(void) {
  if (Vofa_ProcessBusScan()) {
    return;
  }

  if (scope_buf.tail == scope_buf.head) {
    return;
  }

  if (s_status.scope_enabled) {
    float *data = scope_buf.data[scope_buf.tail];
    uint8_t i = 0;

    for (i = 0; i < SCOPE_CHANNELS; ++i) {
      vofa_send_data(i, data[i]);
    }
    vofa_sendframetail();
  }

  scope_buf.tail = (uint16_t)((scope_buf.tail + 1U) % SCOPE_BUFFER_SIZE);
}

void Studio_PeriodicUpdate(void) {
  bool calibrating = false;
  uint32_t fault_bits = 0;
  uint32_t new_faults = 0;
  uint32_t cleared_faults = 0;
  uint32_t i = 0;
  bool cogging_active = false;

  if (!s_status.version_sent) {
    Studio_ReportVersion();
    s_status.version_sent = true;
  }

  calibrating =
      (motor_data.state.State_Mode == STATE_MODE_DETECTING) &&
      (motor_data.state.Sub_State != SUB_STATE_IDLE);
  if (calibrating) {
    int step = Studio_CsStateToStep(motor_data.state.Cs_State);
    if (step != s_status.last_calib_step) {
      s_status.last_calib_step = step;
      Studio_SendTextf("calib_step=%d", step);
    }
    s_status.last_calib_active = true;
  } else if (s_status.last_calib_active) {
    s_status.last_calib_active = false;
    s_status.last_calib_step = -1;
    if (motor_data.state.State_Mode == STATE_MODE_RUNNING) {
      Studio_SendText("calib_done=1");
      Studio_SendTextf("param=%u,%.6g", (unsigned)PARAM_MOTOR_RS,
                       motor_data.parameters.Rs);
      Studio_SendTextf("param=%u,%.6g", (unsigned)PARAM_MOTOR_LS,
                       motor_data.parameters.Ls);
      Studio_SendTextf("param=%u,%.6g", (unsigned)PARAM_MOTOR_FLUX,
                       motor_data.parameters.flux);
      Studio_SendTextf("param=%u,%d", (unsigned)PARAM_MOTOR_POLE_PAIRS,
                       motor_data.parameters.pole_pairs);
    } else if (motor_data.state.State_Mode == STATE_MODE_GUARD) {
      Studio_SendText("calib_error=fault_during_calibration");
    } else {
      Studio_SendText("calib_done=1");
    }
  }

  fault_bits = Safety_GetActiveFaultBits();
  new_faults = fault_bits & ~s_status.last_fault_bits;
  cleared_faults = s_status.last_fault_bits & ~fault_bits;
  if (new_faults != 0U) {
    for (i = 0; i < sizeof(s_fault_map) / sizeof(s_fault_map[0]); ++i) {
      if ((new_faults & s_fault_map[i].bit) != 0U) {
        Studio_SendTextf("fault=%s,%s", s_fault_map[i].code,
                         s_fault_map[i].severity);
      }
    }
  }
  if (cleared_faults != 0U && fault_bits == 0U) {
    Studio_SendText("fault_clear=all");
  }
  s_status.last_fault_bits = fault_bits;

  cogging_active = CoggingComp_IsCalibrating();
  if (cogging_active) {
    uint16_t step = CoggingComp_GetCalibStep();
    if (step != s_status.last_cogging_step && (step % 10U == 0U || step == 0U)) {
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

  if (s_status.status_rate_div > 0U) {
    s_status.status_tick++;
    if (s_status.status_tick >= s_status.status_rate_div) {
      s_status.status_tick = 0;
      Studio_ReportMotorStatus();
    }
  }
}

void Vofa_Process(void) { Scope_Process(); }

void vofa_Receive(uint8_t *buf, uint16_t len) {
  uint16_t copy_len = len;
  char *recv_str = (char *)receive_buf;

  if (buf == NULL) {
    return;
  }

  if (copy_len >= MAX_RXBUFFER_SIZE) {
    copy_len = MAX_RXBUFFER_SIZE - 1U;
  }

  memcpy(receive_buf, buf, copy_len);
  receive_buf[copy_len] = '\0';

  if (strstr(recv_str, "handshake=studio") != NULL) {
    Studio_SendTextf("ack=studio,%d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
                     FW_VERSION_PATCH);
    s_status.version_sent = true;
    return;
  }

  if (strstr(recv_str, "get_version") != NULL) {
    Studio_ReportVersion();
    return;
  }

  if (strstr(recv_str, "get_param=") != NULL) {
    Studio_ReadParam((uint16_t)vofa_cmd_parse_int(recv_str, "get_param="));
    return;
  }

  if (strstr(recv_str, "set_param=") != NULL) {
    const char *payload = strstr(recv_str, "set_param=") + 10;
    int idx = 0;
    float value = 0.0f;

    if (sscanf(payload, "%d,%f", &idx, &value) == 2) {
      Studio_WriteParam((uint16_t)idx, value);
    }
    return;
  }

  if (strstr(recv_str, "read_all_params") != NULL) {
    Studio_ReadAllParams();
    return;
  }

  if (strstr(recv_str, "save_flash=1") != NULL) {
    Param_ScheduleSave();
    Studio_SendText("ack=save_ok");
    return;
  }

  if (strstr(recv_str, "motor_enable=") != NULL) {
    if (vofa_cmd_parse(recv_str, "motor_enable=") > 0.5f) {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_OPERATION_ENABLED);
    } else {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
    return;
  }

  if (strstr(recv_str, "calib=") != NULL) {
    if (vofa_cmd_parse(recv_str, "calib=") > 0.5f) {
      s_status.last_calib_active = false;
      s_status.last_calib_step = -1;
      StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
      Studio_SendText("calib_step=0");
    }
    return;
  }

  if (strstr(recv_str, "clear_fault=1") != NULL) {
    Safety_ClearFaults(&g_ds402_state_machine);
    Motor_ClearFaults(&motor_data);
    s_status.last_fault_bits = 0;
    Studio_SendText("fault_clear=all");
    return;
  }

  if (strstr(recv_str, "set_zero=1") != NULL) {
    float current_pos = MHAL_Encoder_GetPosition();
    MHAL_Encoder_SetOffset(-current_pos);
    Param_ScheduleSave();
    Studio_SendText("ack=zero_set");
    return;
  }

  if (strstr(recv_str, "set_ctrl_mode=") != NULL) {
    int ctrl_mode = vofa_cmd_parse_int(recv_str, "set_ctrl_mode=");
    if (ctrl_mode >= CONTROL_MODE_OPEN && ctrl_mode <= CONTROL_MODE_MIT) {
      motor_data.state.Control_Mode = (CONTROL_MODE)ctrl_mode;
    }
    return;
  }

  if (strstr(recv_str, "set_Iq=") != NULL) {
    motor_data.algo_input.Iq_ref = vofa_cmd_parse(recv_str, "set_Iq=");
    return;
  }

  if (strstr(recv_str, "set_Id=") != NULL) {
    motor_data.algo_input.Id_ref = vofa_cmd_parse(recv_str, "set_Id=");
    return;
  }

  if (strstr(recv_str, "set_torque=") != NULL) {
    motor_data.Controller.input_torque = vofa_cmd_parse(recv_str, "set_torque=");
    return;
  }

  if (strstr(recv_str, "set_vel=") != NULL) {
    motor_data.Controller.input_velocity = vofa_cmd_parse(recv_str, "set_vel=");
    return;
  }

  if (strstr(recv_str, "set_pos=") != NULL) {
    motor_data.Controller.input_position = vofa_cmd_parse(recv_str, "set_pos=");
    motor_data.Controller.input_updated = true;
    return;
  }

  if (strstr(recv_str, "set_current_ctrl_bw=") != NULL) {
    motor_data.Controller.current_ctrl_bandwidth =
        vofa_cmd_parse_int(recv_str, "set_current_ctrl_bw=");
    CurrentLoop_UpdateGain(&motor_data);
    motor_data.params_updated = true;
    return;
  }

  if (strstr(recv_str, "set_vel_kp=") != NULL) {
    motor_data.VelPID.Kp = vofa_cmd_parse(recv_str, "set_vel_kp=");
    return;
  }

  if (strstr(recv_str, "set_vel_ki=") != NULL) {
    motor_data.VelPID.Ki = vofa_cmd_parse(recv_str, "set_vel_ki=");
    return;
  }

  if (strstr(recv_str, "set_pos_kp=") != NULL) {
    motor_data.PosPID.Kp = vofa_cmd_parse(recv_str, "set_pos_kp=");
    return;
  }

  if (strstr(recv_str, "set_ladrc_en=") != NULL) {
    motor_data.ladrc_enable = vofa_cmd_parse(recv_str, "set_ladrc_en=");
    if (motor_data.ladrc_enable >= 0.5f) {
      LADRC_Reset(&motor_data.ladrc_state);
      LADRC_Init(&motor_data.ladrc_state, &motor_data.ladrc_config);
    } else {
      PID_clear(&motor_data.VelPID);
    }
    return;
  }

  if (strstr(recv_str, "set_ladrc_wo=") != NULL) {
    motor_data.ladrc_config.omega_o = vofa_cmd_parse(recv_str, "set_ladrc_wo=");
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    return;
  }

  if (strstr(recv_str, "set_ladrc_wc=") != NULL) {
    motor_data.ladrc_config.omega_c = vofa_cmd_parse(recv_str, "set_ladrc_wc=");
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    return;
  }

  if (strstr(recv_str, "set_ladrc_b0=") != NULL) {
    motor_data.ladrc_config.b0 = vofa_cmd_parse(recv_str, "set_ladrc_b0=");
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    return;
  }

  if (strstr(recv_str, "set_ladrc_max=") != NULL) {
    motor_data.ladrc_config.max_output =
        vofa_cmd_parse(recv_str, "set_ladrc_max=");
    LADRC_UpdateGains(&motor_data.ladrc_state, &motor_data.ladrc_config);
    return;
  }

  if (strstr(recv_str, "set_cogging_calib=1") != NULL) {
    motor_data.advanced.cogging_calib_request = 1.0f;
    Studio_SendText("ack=cogging_calib_started");
    return;
  }

  if (strstr(recv_str, "set_cogging_enable=") != NULL) {
    motor_data.advanced.cogging_comp_enabled =
        vofa_cmd_parse(recv_str, "set_cogging_enable=");
    return;
  }

  if (strstr(recv_str, "set_status_rate=") != NULL) {
    int rate = vofa_cmd_parse_int(recv_str, "set_status_rate=");
    s_status.status_rate_div = (uint8_t)CLAMP(rate, 0, 100);
    s_status.status_tick = 0;
    Studio_SendTextf("ack=status_rate,%d", (int)s_status.status_rate_div);
    return;
  }

  if (strstr(recv_str, "set_scope_enable=") != NULL) {
    s_status.scope_enabled =
        vofa_cmd_parse(recv_str, "set_scope_enable=") > 0.5f;
    Studio_SendTextf("ack=scope_enable,%d", s_status.scope_enabled ? 1 : 0);
    return;
  }

  if (strstr(recv_str, "get_status") != NULL) {
    Studio_ReportMotorStatus();
    return;
  }

  if (strstr(recv_str, "scan_bus=1") != NULL) {
    Vofa_BeginBusScan();
    return;
  }

  if (strstr(recv_str, "boot_enter") != NULL) {
    StateMachine_RequestState(&g_ds402_state_machine,
                              STATE_SWITCH_ON_DISABLED);
    Studio_SendText("boot_ack,0,entering_bootloader");
    HAL_Delay(100);
    Boot_RequestUpgrade();
  }
}
