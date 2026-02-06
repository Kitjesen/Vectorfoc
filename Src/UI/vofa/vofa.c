#include "vofa.h"
#include "motor.h"                 // For motor_data and g_ds402_state_machine
#include "control/control.h" // For CurrentLoop_UpdateGain
#include "usbd_cdc_if.h"

#define MAX_TXBUFFER_SIZE 512        // 定义发送缓冲区大小
#define MAX_RXBUFFER_SIZE 512        // 定义接收缓冲区大小
uint8_t send_buf[MAX_TXBUFFER_SIZE]; // 发送缓冲区
uint8_t receive_buf[MAX_RXBUFFER_SIZE];
uint16_t cnt = 0; // 当前缓冲区计数

/***********************************************************************
 * @brief:      vofa_start(void)
 * @param:      void
 * @retval:     void
 * @details:    发送数据给上位机
 ***********************************************************************/
void vofa_start(void) {
  Vofa_Packet(); // 生成并发送数据包
}

/***********************************************************************
 * @brief:      vofa_transmit(const uint8_t* buf, uint16_t len)
 * @param:      buf: 数据缓冲区指针
 * @param:      len: 数据长度
 * @retval:     void
 * @details:    发送数据到上位机，使用 USART 或 USB
 ***********************************************************************/
#define USART_OR_CDC 1 // 0: 使用 USART 发送，1: 使用 USB 发送
void vofa_transmit(const uint8_t *buf, uint16_t len) {
  if (len > 0 && len <= MAX_TXBUFFER_SIZE) // 检查长度
  {
#if USART_OR_CDC == 0
    memcpy(send_buf, buf, len);
    HAL_UART_Transmit_DMA(&huart3, send_buf, len); // 使用 DMA 发送数据
#elif USART_OR_CDC == 1
    CDC_Transmit_FS((uint8_t *)buf, len);
#endif
  }
}

/***********************************************************************
 * @brief:      vofa_send_data(uint8_t num, float data)
 * @param[in]:   num: 数据编号
 * @param[in]:   data: 浮点数据
 * @retval:     void
 * @details:    将浮点数据拆分成单字节并存储到发送缓冲区
 ***********************************************************************/
void vofa_send_data(uint8_t num, float data) {
  if (cnt + 4 < MAX_TXBUFFER_SIZE) { // 检查缓冲区是否足够
    send_buf[cnt++] = byte0(data);   // 拆分浮点数为字节
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
  }
}

/***********************************************************************
 * @brief:      vofa_sendframetail(void)
 * @param:      NULL
 * @retval:     void
 * @details:    给数据包发送帧尾并发送整个数据包
 ***********************************************************************/
void vofa_sendframetail(void) {
  if (cnt + 4 < MAX_TXBUFFER_SIZE) { // 检查缓冲区是否足够
    send_buf[cnt++] = 0x00;          // 添加帧尾
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;

    vofa_transmit(send_buf, cnt); // 发送整个数据包
    cnt = 0;                      // 重置计数器
  }
}

/***********************************************************************
 * @brief:      Vofa_Packet(void)
 * @param:      NULL
 * @retval:     void
 * @details:    生成并发送数据包
 ***********************************************************************/
void Vofa_Packet(void) {
  // 使用新架构数据源 (algo_input/output)
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
  vofa_sendframetail(); // 发送帧尾并传输数据包
}

/* Scope Buffer Implementation */
static ScopeBuffer_t scope_buf;

void Scope_Init(void) {
  scope_buf.head = 0;
  scope_buf.tail = 0;
}

void Scope_Update(void) {
  /* ISR Context - Push to buffer */
  uint16_t next_head = (scope_buf.head + 1) % SCOPE_BUFFER_SIZE;

  // Drop if full
  if (next_head == scope_buf.tail) {
    return;
  }

  // Sample Data (Matching Vofa_Packet order)
  float *data = scope_buf.data[scope_buf.head];

  // 使用新架构数据源
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
  /* Main Loop Context - Pop from buffer */
  if (scope_buf.tail != scope_buf.head) {
    float *data = scope_buf.data[scope_buf.tail];

    // Send data
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

    scope_buf.tail = (scope_buf.tail + 1) % SCOPE_BUFFER_SIZE;
  }
}

/**USB信息处理**/
static float vofa_cmd_parse(uint8_t *cmdBuf, const char *arg) {
  return atof((char *)cmdBuf + strlen(arg));
}

/**接收中断服务函数**/
void vofa_Receive(uint8_t *buf, uint16_t len) {
  memcpy((char *)receive_buf, buf, len);

  char *recvStr = (char *)receive_buf;

  if (strstr(recvStr, "set_Iq=")) {
    motor_data.algo_input.Iq_ref = vofa_cmd_parse(receive_buf, "set_Iq=");
  }

  if (strstr(recvStr, "set_Id=")) {
    motor_data.algo_input.Id_ref = vofa_cmd_parse(receive_buf, "set_Id=");
  }

  if (strstr(recvStr, "set_torque=")) {
    motor_data.Controller.input_torque =
        vofa_cmd_parse(receive_buf, "set_torque=");
  }

  if (strstr(recvStr, "set_vel=")) {
    motor_data.Controller.input_velocity =
        vofa_cmd_parse(receive_buf, "set_vel=");
  }

  if (strstr(recvStr, "set_pos=")) {
    motor_data.Controller.input_position =
        vofa_cmd_parse(receive_buf, "set_pos=");
    motor_data.Controller.input_updated = true;
  }

  if (strstr(recvStr, "set_current_ctrl_bw=")) {
    motor_data.Controller.current_ctrl_bandwidth =
        vofa_cmd_parse(receive_buf, "set_current_ctrl_bw=");
    // Update gains based on new bandwidth
    CurrentLoop_UpdateGain(&motor_data);
    // Flag update for inner loop
    motor_data.params_updated = true;
  }

  if (strstr(recvStr, "set_vel_kp=")) {
    motor_data.VelPID.Kp = vofa_cmd_parse(receive_buf, "set_vel_kp=");
  }

  if (strstr(recvStr, "set_vel_ki=")) {
    motor_data.VelPID.Ki = vofa_cmd_parse(receive_buf, "set_vel_ki=");
  }

  if (strstr(recvStr, "set_pos_kp=")) {
    motor_data.PosPID.Kp = vofa_cmd_parse(receive_buf, "set_pos_kp=");
  }

  if (strstr(recvStr, "set_ctrl_mode=")) {
    float ctrlModeVal = vofa_cmd_parse(receive_buf, "set_ctrl_mode=");
    int ctrlModeInt = (int)ctrlModeVal;
    if (ctrlModeInt >= CONTROL_MODE_OPEN &&
        ctrlModeInt <= CONTROL_MODE_POSITION_RAMP) {
      motor_data.state.Control_Mode = (CONTROL_MODE)ctrlModeInt;
    }
  }

  if (strstr(recvStr, "calib=")) {
    float calib_enable = vofa_cmd_parse(receive_buf, "calib=");
    if (calib_enable == 1) {
      // [Modified for FSM Integration] Request via FSM
      StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
    }
  }

  if (strstr(recvStr, "motor_enable=")) {
    float motor_enable = vofa_cmd_parse(receive_buf, "motor_enable=");
    if (motor_enable > 0.5f) {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_OPERATION_ENABLED);
    } else {
      StateMachine_RequestState(&g_ds402_state_machine,
                                STATE_SWITCH_ON_DISABLED);
    }
  }
}
