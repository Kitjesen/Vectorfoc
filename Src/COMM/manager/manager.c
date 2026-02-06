/**
 * @file manager.c
 * @brief 通信协议管理器 - 统一路由和协议切换
 *
 * 职责:
 *   1. 多协议支持: Inovxio/CANopen/MIT三种协议动态切换
 *   2. 统一路由: 根据当前协议自动分发到对应处理函数
 *   3. 应用集成: 将协议命令转换为电机控制动作
 *
 * 架构:
 *   CAN接收 → Protocol_ProcessRxFrame() → 协议解析 → 电机控制
 *                                       ↓
 *   CAN发送 ← Protocol_BuildFeedback() ← 状态读取
 *
 * 使用示例:
 *   Protocol_Init(PROTOCOL_INOVXIO);        // 初始化
 *   Protocol_ProcessRxFrame(&can_frame);    // 在CAN中断中调用
 *   Protocol_SendFrame(&can_frame);         // 发送CAN帧

  系统初始化 和 CAN 接收中断。
 */
#include "main.h" // For FDCAN support

#include "manager.h"
#include "transport.h"

#include "bsp_can.h" // 仍需包含以提供默认 CAN 传输
#include "bsp_dwt.h"
#include "protocol/canopen/canopen_protocol.h"
#include "error_manager.h"
#include "error_types.h"
#include "executor/executor.h"
#include "fsm.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"
#include "protocol/inovxio/inovxio_protocol.h"
#include "protocol/mit/mit_protocol.h"
#include "motor.h"
#include "safety_control.h" // For Safety_GetActiveFaultBits
#include "param_access.h"
#include "param_table.h"
#include <string.h>

#define CAN_BROADCAST_ADDR 0x7F   ///< CAN广播地址
#define INOVXIO_CMD_GET_ID 0x00   ///< Inovxio协议获取ID命令
#define PROTOCOL_RX_QUEUE_LEN 32U ///< Rx ring buffer length

/* 当前协议类型 */
static ProtocolType s_current_protocol = PROTOCOL_INOVXIO;

/* 传输层接口（默认为 NULL，使用 BSP_CAN 作为回退）*/
static const TransportInterface *s_transport = NULL;

/* ISR->Task Rx ring buffer */
static CAN_Frame s_rx_queue[PROTOCOL_RX_QUEUE_LEN];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;
static volatile uint32_t s_rx_dropped = 0;
static volatile bool s_rx_overflow = false;

/* Communication statistics */
static CommStats_t s_comm_stats = {0};
static volatile uint32_t s_rx_queue_peak = 0;

static bool Protocol_DequeueRxFrame(CAN_Frame *out) {
  if (out == NULL) {
    return false;
  }

  bool has_frame = false;
  __disable_irq();
  if (s_rx_head != s_rx_tail) {
    *out = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % PROTOCOL_RX_QUEUE_LEN);
    has_frame = true;
  }
  __enable_irq();

  return has_frame;
}

/**
 * @brief 注册传输层接口
 */
void Protocol_RegisterTransport(const TransportInterface *transport) {
  s_transport = transport;
}

/**
 * @brief 初始化协议管理器
 */
void Protocol_Init(ProtocolType default_protocol) {
  s_current_protocol = default_protocol;

  // 初始化各协议模块
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    ProtocolPrivate_Init();
    break;
  case PROTOCOL_CANOPEN:
    ProtocolCANopen_Init();
    break;
  case PROTOCOL_MIT:
    ProtocolMIT_Init();
    break;
  default:
    // 默认使用Inovxio协议
    s_current_protocol = PROTOCOL_INOVXIO;
    ProtocolPrivate_Init();
    break;
  }
}

/**
 * @brief 设置协议类型
 */
void Protocol_SetType(ProtocolType protocol) {
  if (protocol != s_current_protocol) {
    s_current_protocol = protocol;
    Protocol_Init(protocol);
  }
}

/**
 * @brief 获取当前协议类型
 */
ProtocolType Protocol_GetType(void) { return s_current_protocol; }

/**
 * @brief 解析CAN帧 (调用对应协议的解析函数)
 */
ParseResult Protocol_ParseFrame(const CAN_Frame *frame, MotorCommand *cmd) {
  if (frame == NULL || cmd == NULL) {
    ERROR_REPORT(ERROR_COMM_INVALID_FRAME, "Invalid CAN frame");
    return PARSE_ERR_INVALID_FRAME;
  }

  ParseResult result;
  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    result = ProtocolPrivate_Parse(frame, cmd);
    break;

  case PROTOCOL_CANOPEN:
    result = ProtocolCANopen_Parse(frame, cmd);
    break;

  case PROTOCOL_MIT:
    result = ProtocolMIT_Parse(frame, cmd);
    break;

  default:
    result = PARSE_ERR_INVALID_FRAME;
    break;
  }

  // 报告解析错误（除了OK之外的错误，UNKNOWN_ID很常见可忽略）
  if (result != PARSE_OK && result != PARSE_UNKNOWN_ID) {
    s_comm_stats.parse_errors++;
    ERROR_REPORT(ERROR_COMM_PARSE_FAILED, "CAN frame parse error");
  }

  return result;
}

/**
 * @brief 构建反馈帧
 */
bool Protocol_BuildFeedback(const MotorStatus *status, CAN_Frame *frame) {
  if (status == NULL || frame == NULL) {
    return false;
  }

  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildFeedback(status, frame);

  case PROTOCOL_CANOPEN:
    return ProtocolCANopen_BuildFeedback(status, frame);

  case PROTOCOL_MIT:
    return ProtocolMIT_BuildFeedback(status, frame);

  default:
    return false;
  }
}

/**
 * @brief 构建故障帧
 */
bool Protocol_BuildFault(uint32_t fault_code, CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildFault(fault_code, 0, frame);

  case PROTOCOL_CANOPEN:
    return ProtocolCANopen_BuildFault(fault_code, frame);

  case PROTOCOL_MIT:
    return ProtocolMIT_BuildFault(fault_code, frame);

  default:
    return false;
  }
}

/**
 * @brief 构建参数响应帧
 */
bool Protocol_BuildParamResponse(uint16_t param_index, float value,
                                 CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  switch (s_current_protocol) {
  case PROTOCOL_INOVXIO:
    return ProtocolPrivate_BuildParamResponse(param_index, value, frame);

    // 其他协议如果支持参数读取响应，在这里添加
    // case PROTOCOL_CANOPEN: ...

  default:
    return false;
  }
}

/**
 * @brief 发送CAN帧
 * @note 优先使用注册的传输接口，若未注册则回退到 BSP_CAN_SendFrame
 */
bool Protocol_SendFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  bool result = false;

  /* 优先使用传输层接口 */
  if (s_transport != NULL && s_transport->send != NULL) {
    TransportFrame tf;
    Transport_FromCANFrame(frame, &tf);
    result = s_transport->send(&tf);
  } else {
    /* 回退到 BSP CAN 直接调用（兼容旧代码）*/
    result = BSP_CAN_SendFrame(frame);
  }

  if (result) {
    s_comm_stats.tx_frames_total++;
  } else {
    s_comm_stats.tx_frames_failed++;
  }
  return result;
}

/**
 * @brief 进入Rx队列 (ISR安全)
 */
bool Protocol_QueueRxFrame(const CAN_Frame *frame) {
  if (frame == NULL) {
    return false;
  }

  uint8_t next = (uint8_t)((s_rx_head + 1U) % PROTOCOL_RX_QUEUE_LEN);
  if (next == s_rx_tail) {
    s_rx_dropped++;
    s_rx_overflow = true;
    s_comm_stats.rx_frames_dropped++;
    return false;
  }

  s_rx_queue[s_rx_head] = *frame;
  s_rx_head = next;
  s_comm_stats.rx_frames_total++;

  // Update queue depth statistics
  uint8_t depth = (s_rx_head >= s_rx_tail)
                      ? (s_rx_head - s_rx_tail)
                      : (PROTOCOL_RX_QUEUE_LEN - s_rx_tail + s_rx_head);
  if (depth > s_rx_queue_peak) {
    s_rx_queue_peak = depth;
  }

  return true;
}

/**
 * @brief 在任务中处理队列中的CAN帧
 */
void Protocol_ProcessQueuedFrames(void) {
  CAN_Frame frame;
  if (s_rx_overflow) {
    s_rx_overflow = false;
    s_comm_stats.rx_overflow_events++;
    ErrorManager_Report(ERROR_COMM_INVALID_FRAME, "Rx queue overflow");
  }
  while (Protocol_DequeueRxFrame(&frame)) {
    Protocol_ProcessRxFrame(&frame);
  }
}

/**
 * @brief 处理接收到的CAN帧
 * @note 这是通信协议的核心调度入口，建议在任务中调用；
 *       ISR 内仅做 Protocol_QueueRxFrame() 入队
 */
void Protocol_ProcessRxFrame(const CAN_Frame *frame) {
  if (frame == NULL)
    return;

#ifdef DEBUG
  static uint32_t start_cnt = 0;
  uint32_t exec_time_us;
  start_cnt = DWT->CYCCNT;
#endif

  MotorCommand cmd;

  // 0. 特殊处理: INOVXIO协议的设备发现 (GET_ID)
  // 这是协议外的广播命令，用于上位机发现总线上的所有电机
  if (s_current_protocol == PROTOCOL_INOVXIO) {
    uint8_t cmd_type = (frame->id >> 24) & 0x1F;
    if (cmd_type == INOVXIO_CMD_GET_ID) {
      uint8_t target = frame->id & 0xFF;
      // 仅响应广播或匹配ID的命令
      if (target == g_can_id || target == CAN_BROADCAST_ADDR) {
        CAN_Frame tx_frame;
        // 响应格式: [0][MyID][FE][UUID-Low32]
        // 这里返回UUID的前32位，用于生成唯一的设备标识
        tx_frame.id = (0x00 << 24) | (g_can_id << 8) | 0xFE;
        tx_frame.is_extended = true;
        tx_frame.dlc = 8;

        // 读取STM32唯一ID (12字节/96位)
        uint32_t uuid[3] = {0x12345678, 0x87654321, 0x00000000};
        uint32_t *uid = (uint32_t *)UID_BASE; // UID_BASE defined in STM32 HAL
        memcpy(uuid, uid, 12);
        memcpy(tx_frame.data, uuid, 8); // 仅发送前8字节

        Protocol_SendFrame(&tx_frame);
        return; // 处理完毕直接返回，不解析为常规命令
      }
    }
  }

  // 1. 协议解析: 根据当前激活的协议解析CAN帧
  ParseResult result = Protocol_ParseFrame(frame, &cmd);

  // 2. 命令执行: 将解析后的统一 MotorCommand 交给执行器
  if (result == PARSE_OK) {
    Executor_ProcessCommand(&cmd);
  }

#ifdef DEBUG
  // Calculate execution time (assuming 168MHz CPU)
  exec_time_us = (DWT->CYCCNT - start_cnt) / 168;
  if (exec_time_us > s_comm_stats.exec_time_max_us) {
    s_comm_stats.exec_time_max_us = exec_time_us;
  }
#endif
}
/**
 * @brief  Callback for Safety Module to report faults via CAN.
 */
bool Protocol_ReportFaultCallback(uint32_t fault_bits, MOTOR_DATA *motor) {
  CAN_Frame tx_frame;
  // Build fault frame (CMD 21)
  if (Protocol_BuildFault(fault_bits, &tx_frame)) {
    // Send frame (returns true on success, false on TX full)
    return Protocol_SendFrame(&tx_frame);
  }
  return false;
}

void Protocol_PeriodicUpdate(uint32_t now_ms, const MotorStatus *status) {
  (void)status;

  if (s_current_protocol == PROTOCOL_CANOPEN) {
    CAN_Frame hb_frame;
    if (ProtocolCANopen_BuildHeartbeat(now_ms, &hb_frame)) {
      Protocol_SendFrame(&hb_frame);
    }
  }
}

/**
 * @brief Get communication statistics
 */
void Protocol_GetStats(CommStats_t *stats) {
  if (stats == NULL) {
    return;
  }

  __disable_irq();
  *stats = s_comm_stats;
  
  // Calculate current queue depth
  uint8_t depth = (s_rx_head >= s_rx_tail)
                      ? (s_rx_head - s_rx_tail)
                      : (PROTOCOL_RX_QUEUE_LEN - s_rx_tail + s_rx_head);
  stats->rx_queue_depth = depth;
  stats->rx_queue_peak = s_rx_queue_peak;
  __enable_irq();
}

/**
 * @brief Reset communication statistics
 */
void Protocol_ResetStats(void) {
  __disable_irq();
  s_comm_stats.rx_frames_total = 0;
  s_comm_stats.rx_frames_dropped = 0;
  s_comm_stats.rx_overflow_events = 0;
  s_comm_stats.tx_frames_total = 0;
  s_comm_stats.tx_frames_failed = 0;
  s_comm_stats.parse_errors = 0;
  s_comm_stats.exec_time_max_us = 0;
  s_rx_queue_peak = 0;
  __enable_irq();
}
