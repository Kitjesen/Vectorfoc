/**
 * @file comm_stats_example.c
 * @brief 通信统计使用示例
 * 
 * 本文件展示如何在应用中集成通信统计功能。
 * 可以将相关代码复制到 task_comm.c 或其他需要监控通信的地方。
 */

#include "manager.h"
#include "bsp_log.h"  // 或其他日志输出接口
#include <stdio.h>

/**
 * @brief 周期性输出通信统计（建议在通信任务中调用）
 * @param force 是否强制输出（忽略时间间隔）
 */
void CommStats_PeriodicLog(bool force) {
  static uint32_t last_log_time = 0;
  static CommStats_t last_stats = {0};
  
  uint32_t now = HAL_GetTick();
  
  // 每秒输出一次，或强制输出
  if (!force && (now - last_log_time < 1000)) {
    return;
  }
  
  CommStats_t stats;
  Protocol_GetStats(&stats);
  
  // 计算增量统计（每秒的帧数）
  uint32_t rx_per_sec = stats.rx_frames_total - last_stats.rx_frames_total;
  uint32_t tx_per_sec = stats.tx_frames_total - last_stats.tx_frames_total;
  uint32_t dropped_per_sec = stats.rx_frames_dropped - last_stats.rx_frames_dropped;
  
  // 输出统计信息
  LOG_INFO("=== CAN Communication Statistics ===");
  LOG_INFO("RX: %lu frames/s (total: %lu, dropped: %lu, overflow: %lu events)",
           rx_per_sec, stats.rx_frames_total, stats.rx_frames_dropped, 
           stats.rx_overflow_events);
  LOG_INFO("TX: %lu frames/s (total: %lu, failed: %lu)",
           tx_per_sec, stats.tx_frames_total, stats.tx_frames_failed);
  LOG_INFO("Queue: depth=%lu, peak=%lu/%u",
           stats.rx_queue_depth, stats.rx_queue_peak, 32);
  LOG_INFO("Parse errors: %lu, Max exec time: %lu us",
           stats.parse_errors, stats.exec_time_max_us);
  
  // 告警检测
  if (stats.rx_frames_dropped > last_stats.rx_frames_dropped) {
    LOG_WARN("CAN RX queue overflow detected! Consider increasing queue size.");
  }
  
  if (stats.rx_queue_peak > 28) {  // 队列长度 32，告警阈值 28 (87.5%)
    LOG_WARN("CAN RX queue near full! Peak: %lu/32", stats.rx_queue_peak);
  }
  
  if (stats.exec_time_max_us > 100) {  // 处理时间超过 100us
    LOG_WARN("CAN frame processing time too long: %lu us", stats.exec_time_max_us);
  }
  
  // 保存当前统计用于下次计算增量
  last_stats = stats;
  last_log_time = now;
}

/**
 * @brief 在 VOFA+ 中可视化统计（在主循环或调试任务中调用）
 */
void CommStats_VofaOutput(void) {
  static uint32_t last_output_time = 0;
  uint32_t now = HAL_GetTick();
  
  // VOFA+ 建议采样率 50-100Hz
  if (now - last_output_time < 20) {  // 20ms = 50Hz
    return;
  }
  
  CommStats_t stats;
  Protocol_GetStats(&stats);
  
  // 计算帧率（每秒帧数）
  static uint32_t last_rx_total = 0;
  static uint32_t last_tx_total = 0;
  float rx_rate = (stats.rx_frames_total - last_rx_total) * 50.0f;  // 50Hz采样
  float tx_rate = (stats.tx_frames_total - last_tx_total) * 50.0f;
  
  // VOFA+ FireWater 协议输出（假设有 vofa_send 函数）
  float vofa_data[8] = {
    rx_rate,                           // Ch0: RX 帧率
    tx_rate,                           // Ch1: TX 帧率
    (float)stats.rx_queue_depth,       // Ch2: 队列深度
    (float)stats.rx_queue_peak,        // Ch3: 队列峰值
    (float)stats.rx_frames_dropped,    // Ch4: 丢帧总数
    (float)stats.parse_errors,         // Ch5: 解析错误
    (float)stats.exec_time_max_us,     // Ch6: 最大处理时间
    (float)stats.tx_frames_failed,     // Ch7: 发送失败
  };
  
  // vofa_send(vofa_data, 8);  // 需要实现或使用现有 VOFA 模块
  
  last_rx_total = stats.rx_frames_total;
  last_tx_total = stats.tx_frames_total;
  last_output_time = now;
}

/**
 * @brief 集成示例：在通信任务中使用
 * 
 * 将以下代码添加到 task_comm.c 的 StartCustomTask 函数中：
 * 
 * ```c
 * void StartCustomTask(void const *argument) {
 *   (void)argument;
 *   CmdService_Init();
 *   
 *   for (;;) {
 *     Protocol_ProcessQueuedFrames();
 *     CmdService_Process();
 *     
 *     // 添加统计输出
 *     CommStats_PeriodicLog(false);
 *     // 或在 VOFA+ 中可视化
 *     // CommStats_VofaOutput();
 *     
 *     osDelay(2);
 *   }
 * }
 * ```
 */

/**
 * @brief 命令行接口示例：通过串口命令查询统计
 * 
 * 如果有命令行接口（如 cmd_service），可以添加以下命令：
 */
void CmdHandler_CanStats(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  CommStats_t stats;
  Protocol_GetStats(&stats);
  
  printf("\r\n=== CAN Communication Statistics ===\r\n");
  printf("RX Frames:        %10lu (%.2f frames/s)\r\n", 
         stats.rx_frames_total, stats.rx_frames_total / (HAL_GetTick() / 1000.0f));
  printf("RX Dropped:       %10lu (%.3f%%)\r\n", 
         stats.rx_frames_dropped, 
         stats.rx_frames_total > 0 ? 100.0f * stats.rx_frames_dropped / stats.rx_frames_total : 0.0f);
  printf("RX Overflow:      %10lu events\r\n", stats.rx_overflow_events);
  printf("RX Queue Depth:   %10lu (peak: %lu)\r\n", 
         stats.rx_queue_depth, stats.rx_queue_peak);
  printf("TX Frames:        %10lu (%.2f frames/s)\r\n", 
         stats.tx_frames_total, stats.tx_frames_total / (HAL_GetTick() / 1000.0f));
  printf("TX Failed:        %10lu (%.3f%%)\r\n", 
         stats.tx_frames_failed,
         stats.tx_frames_total > 0 ? 100.0f * stats.tx_frames_failed / stats.tx_frames_total : 0.0f);
  printf("Parse Errors:     %10lu\r\n", stats.parse_errors);
  printf("Max Exec Time:    %10lu us\r\n", stats.exec_time_max_us);
  printf("\r\n");
  
  // 注册命令（伪代码）：
  // CMD_REGISTER("can_stats", CmdHandler_CanStats, "Display CAN statistics");
}

/**
 * @brief 统计重置命令
 */
void CmdHandler_CanStatsReset(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  Protocol_ResetStats();
  printf("CAN statistics reset.\r\n");
  
  // CMD_REGISTER("can_stats_reset", CmdHandler_CanStatsReset, "Reset CAN statistics");
}
