#include "vofa.h"
#include "control/control.h"
#include "device_id.h"
#include "inovxio_protocol.h"
#include "manager.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include "vofa_bus_scan.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TXBUFFER_SIZE 512
#define MAX_RXBUFFER_SIZE 512
#define TEXT_LINE_MAX 128
#define VOFA_SCAN_TIMEOUT_MS 120U
#define VOFA_SCAN_BROADCAST_ID 0x7FU

static uint8_t send_buf[MAX_TXBUFFER_SIZE];
static uint8_t receive_buf[MAX_RXBUFFER_SIZE];
static uint16_t cnt = 0;
static ScopeBuffer_t scope_buf;
static VofaBusScanState s_bus_scan;

static void vofa_transmit(const uint8_t *buf, uint16_t len) {
  if (len == 0 || len > MAX_TXBUFFER_SIZE) {
    return;
  }

  CDC_Transmit_FS((uint8_t *)buf, len);
}

static void vofa_send_text(const char *text) {
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

static void vofa_send_textf(const char *fmt, ...) {
  char text_buf[TEXT_LINE_MAX + 1];
  va_list args;
  int written = 0;

  va_start(args, fmt);
  written = vsnprintf(text_buf, sizeof(text_buf), fmt, args);
  va_end(args);

  if (written <= 0) {
    return;
  }

  vofa_send_text(text_buf);
}

static void Vofa_BusScanObserver(const CAN_Frame *frame) {
  VofaBusScan_ObserveFrame(&s_bus_scan, frame);
}

static bool Vofa_ProcessBusScan(void) {
  VofaBusScanEvent event = {0};

  if (!VofaBusScan_IsBusy(&s_bus_scan)) {
    return false;
  }

  if (VofaBusScan_PollEvent(&s_bus_scan, HAL_GetTick(), &event)) {
    if (event.type == VOFA_BUS_SCAN_EVENT_NODE) {
      vofa_send_textf("bus_node=%u,online,%08lX%08lX",
                      (unsigned)event.node.node_id,
                      (unsigned long)event.node.uid_word0,
                      (unsigned long)event.node.uid_word1);
    } else if (event.type == VOFA_BUS_SCAN_EVENT_DONE) {
      vofa_send_textf("bus_scan_done,count=%u", (unsigned)event.total_nodes);
    }
  }

  return true;
}

static void Vofa_BeginBusScan(void) {
  CAN_Frame frame = {0};
  DeviceUID_t uid = {0};

  if (Protocol_GetType() != PROTOCOL_INOVXIO) {
    vofa_send_text("bus_scan_error=protocol");
    return;
  }

  DeviceID_GetUID(&uid);
  VofaBusScan_Begin(&s_bus_scan, HAL_GetTick(), VOFA_SCAN_TIMEOUT_MS, g_can_id,
                    uid.word0, uid.word1);
  vofa_send_text("bus_scan_start");

  frame.id = ((uint32_t)PRIVATE_CMD_GET_ID << 24) | VOFA_SCAN_BROADCAST_ID;
  frame.dlc = 0;
  frame.is_extended = true;
  frame.is_rtr = false;

  if (!Protocol_SendFrame(&frame)) {
    vofa_send_text("bus_scan_warn=tx_failed");
  }
}

static float vofa_cmd_parse(const uint8_t *cmd_buf, const char *arg) {
  return atof((const char *)cmd_buf + strlen(arg));
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

  if (scope_buf.tail != scope_buf.head) {
    float *data = scope_buf.data[scope_buf.tail];

    vofa_send_data(0, data[0]);
    vofa_send_data(1, data[1]);
    vofa_send_data(2, data[2]);
    vofa_send_data(3, data[3]);
    vofa_send_data(4, data[4]);
    vofa_send_data(5, data[5]);
    vofa_send_data(6, data[6]);
    vofa_send_data(7, data[7]);
    vofa_send_data(8, data[8]);
    vofa_send_data(9, data[9]);
    vofa_send_data(10, data[10]);
    vofa_send_data(11, data[11]);
    vofa_sendframetail();

    scope_buf.tail = (uint16_t)((scope_buf.tail + 1U) % SCOPE_BUFFER_SIZE);
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

  if (strstr(recv_str, "set_Iq=") != NULL) {
    motor_data.algo_input.Iq_ref = vofa_cmd_parse(receive_buf, "set_Iq=");
  }

  if (strstr(recv_str, "set_Id=") != NULL) {
    motor_data.algo_input.Id_ref = vofa_cmd_parse(receive_buf, "set_Id=");
  }

  if (strstr(recv_str, "set_torque=") != NULL) {
    motor_data.Controller.input_torque =
        vofa_cmd_parse(receive_buf, "set_torque=");
  }

  if (strstr(recv_str, "set_vel=") != NULL) {
    motor_data.Controller.input_velocity = vofa_cmd_parse(receive_buf, "set_vel=");
  }

  if (strstr(recv_str, "set_pos=") != NULL) {
    motor_data.Controller.input_position = vofa_cmd_parse(receive_buf, "set_pos=");
    motor_data.Controller.input_updated = true;
  }

  if (strstr(recv_str, "set_current_ctrl_bw=") != NULL) {
    motor_data.Controller.current_ctrl_bandwidth =
        vofa_cmd_parse(receive_buf, "set_current_ctrl_bw=");
    CurrentLoop_UpdateGain(&motor_data);
    motor_data.params_updated = true;
  }

  if (strstr(recv_str, "set_vel_kp=") != NULL) {
    motor_data.VelPID.Kp = vofa_cmd_parse(receive_buf, "set_vel_kp=");
  }

  if (strstr(recv_str, "set_vel_ki=") != NULL) {
    motor_data.VelPID.Ki = vofa_cmd_parse(receive_buf, "set_vel_ki=");
  }

  if (strstr(recv_str, "set_pos_kp=") != NULL) {
    motor_data.PosPID.Kp = vofa_cmd_parse(receive_buf, "set_pos_kp=");
  }

  if (strstr(recv_str, "set_ctrl_mode=") != NULL) {
    int ctrl_mode = (int)vofa_cmd_parse(receive_buf, "set_ctrl_mode=");
    if (ctrl_mode >= CONTROL_MODE_OPEN &&
        ctrl_mode <= CONTROL_MODE_POSITION_RAMP) {
      motor_data.state.Control_Mode = (CONTROL_MODE)ctrl_mode;
    }
  }

  if (strstr(recv_str, "calib=") != NULL) {
    if (vofa_cmd_parse(receive_buf, "calib=") == 1.0f) {
      StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
    }
  }

  if (strstr(recv_str, "motor_enable=") != NULL) {
    if (vofa_cmd_parse(receive_buf, "motor_enable=") > 0.5f) {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_OPERATION_ENABLED);
    } else {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
  }

  if (strstr(recv_str, "scan_bus=1") != NULL) {
    Vofa_BeginBusScan();
  }
}
