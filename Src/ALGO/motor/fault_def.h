#ifndef FAULT_DEF_H
#define FAULT_DEF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 故障码定义 */
typedef enum {
  FAULT_NONE = 0,

  FAULT_OVER_TEMP = (1 << 0),            // bit0: 电机过温故障 (>145℃)
  FAULT_DRIVER_CHIP = (1 << 1),          // bit1: 驱动芯片故障
  FAULT_UNDER_VOLTAGE = (1 << 2),        // bit2: 欠压故障 (<12V)
  FAULT_OVER_VOLTAGE = (1 << 3),         // bit3: 过压故障 (>60V)
  FAULT_CURRENT_B = (1 << 4),            // bit4: B相电流采样过流
  FAULT_CURRENT_C = (1 << 5),            // bit5: C相电流采样过流
  FAULT_ENCODER_LOSS = (1 << 6),         // bit6: 编码器丢失/通信故障
  FAULT_ENCODER_UNCALIBRATED = (1 << 7), // bit7: 编码器未标定
  FAULT_HARDWARE_ID = (1 << 8),          // bit8: 硬件识别故障
  FAULT_POSITION_INIT = (1 << 9),        // bit9: 位置初始化故障
  FAULT_STALL_OVERLOAD = (1 << 14),      // bit14: 电机堵转过载算法保护
  FAULT_CURRENT_A = (1 << 16),           // bit16: A相电流采样过流
} FaultBit;

#define FAULT_OVER_CURRENT (FAULT_CURRENT_A | FAULT_CURRENT_B | FAULT_CURRENT_C)

/* 警告码定义 */
typedef enum {
  WARNING_NONE = 0,
  WARNING_OVER_TEMP = (1 << 0), // bit0: 电机过温预警
} WarningBit;

/* 故障信息结构 */
typedef struct {
  uint32_t fault_code;      // 故障码
  uint32_t warning_code;    // 警告码
  uint32_t fault_count;     // 累计故障次数
  uint32_t last_fault_time; // 最后故障时间戳
} FaultInfo;

/* 检测状态 - 用于调试查看 */
typedef struct {
  float vbus_filtered;   // 滤波后的母线电压
  float current_peak;    // 峰值电流
  float temp_filtered;   // 滤波后的温度
  float temp_calculated; // 计算温度

  uint32_t stall_counter; // 堵转计数
  bool is_stall;          // 是否堵转

  uint32_t last_can_time; // 最后CAN时间
  bool is_can_timeout;    // 是否CAN超时
  uint32_t encoder_err_consecutive; // 编码器连续错误计数
  uint32_t encoder_err_count;       // 编码器累计错误计数(快照)
} DetectionState;

/* 保护配置参数 */
typedef struct {
  /* 电压保护阈值 */
  float over_voltage_threshold;  // 过压阈值
  float under_voltage_threshold; // 欠压阈值

  /* 电流保护阈值 */
  float over_current_threshold; // 过流阈值

  /* 温度保护阈值 */
  float over_temp_threshold;    // 过温阈值
  float temp_warning_threshold; // 过温预警阈值

  /* CAN通信超时 */
  uint32_t can_timeout_ms; // CAN超时时间

  /* 堵转检测 */
  uint32_t stall_detect_time_ms;  // 堵转检测时间
  float stall_current_threshold;  // 堵转电流阈值
  float stall_velocity_threshold; // 堵转速度阈值

  /* 使能控制 */
  bool enable_voltage_protection;
  bool enable_current_protection;
  bool enable_temp_protection;
  bool enable_stall_protection;
  bool enable_can_timeout;

} DetectionConfig;

/* 故障发生时的回调函数类型 */
typedef bool (*SafetyFaultCallback)(uint32_t fault_code, void *motor);

/* 安全控制配置 */
typedef struct {
  DetectionConfig detection_config;   /**< 检测配置 */
  SafetyFaultCallback fault_callback; /**< 故障回调（可选） */
  bool auto_clear_on_recover;         /**< 故障恢复时自动清除 */
} SafetyConfig;

/* 安全模块内部上下文 */
typedef struct {
  SafetyConfig config;
  uint32_t active_fault_bits;  /**< 当前激活的FaultBit位掩码 */
  uint32_t pending_fault_bits; /**< 待处理/上报的故障位掩码 */
  uint32_t fault_count;        /**< 累计故障次数 */
  uint32_t last_fault_time;    /**< 最后故障时间戳 (ms) */
  bool initialized;
} SafetyContext;

/* 默认保护参数 */
#define FAULT_VBUS_OVERVOLT_V 60.0f
#define FAULT_VBUS_UNDERVOLT_V 12.0f
#define FAULT_OVER_CURRENT_A 90.0f
#define FAULT_TEMP_ERROR_C 145.0f
#define FAULT_TEMP_WARN_C 130.0f
#define FAULT_CAN_TIMEOUT_MS 0
#define FAULT_STALL_DETECT_TIME_MS 500
#define FAULT_STALL_CURRENT_A 80.0f
#define FAULT_STALL_VELOCITY_RAD_S 0.1f

/* 编码器故障检测阈值 */
#define FAULT_ENCODER_ERR_CONSECUTIVE_MAX 20U
#define FAULT_ENCODER_ERR_COUNT_MAX 20U

/* 故障检测内部常量 */
#define FAULT_FILTER_ALPHA_SLOW 0.99f // 母线电压/温度慢速滤波系数
#define FAULT_FILTER_ALPHA_FAST 0.01f // 母线电压/温度快速滤波系数
#define STALL_DETECT_COUNT_PER_MS 20  // 堵转检测计数转换 (20kHz控制频率)

/* 默认检测/安全配置声明 (定义见 fault_def.c) */
extern const DetectionConfig DEFAULT_DETECTION_CONFIG;
extern const SafetyConfig DEFAULT_SAFETY_CONFIG;

#endif /* FAULT_DEF_H */
