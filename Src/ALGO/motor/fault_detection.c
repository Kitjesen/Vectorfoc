#include "fault_detection.h"
#include "hal_abstraction.h"
#include "motor.h"
#include "config.h"
#include "mt6816_encoder.h"
#include <math.h>

DetectionConfig s_config; // Exported for param_table
static DetectionState s_state = {0};

void Detection_Init(const DetectionConfig *config) {
  if (config != NULL) {
    s_config = *config;
  } else {
    s_config = DEFAULT_DETECTION_CONFIG;
  }
  Detection_Reset();
}

void Detection_Reset(void) {
  s_state.stall_counter = 0;
  s_state.is_stall = false;
  s_state.is_can_timeout = false;
  s_state.vbus_filtered = 0.0f;
  s_state.temp_filtered = 25.0f;
  s_state.encoder_err_consecutive = 0;
  s_state.encoder_err_count = 0;
}

/**
 * @brief 电压保护检测
 * @param m 电机数据指针
 * @param vbus 当前母线电压
 * @return 故障码（如有）
 */
static inline uint32_t Detection_CheckVoltage(MOTOR_DATA *m, float vbus) {
  (void)m; // 参数保留用于扩展
  uint32_t fault = FAULT_NONE;

  if (!s_config.enable_voltage_protection)
    return fault;

  // 滤波
  s_state.vbus_filtered = s_state.vbus_filtered * FAULT_FILTER_ALPHA_SLOW +
                          vbus * FAULT_FILTER_ALPHA_FAST;

  // 检测
  if (s_state.vbus_filtered > s_config.over_voltage_threshold)
    fault |= FAULT_OVER_VOLTAGE;
  if (s_state.vbus_filtered < s_config.under_voltage_threshold)
    fault |= FAULT_UNDER_VOLTAGE;

  return fault;
}

/**
 * @brief 电流保护检测
 * @param m 电机数据指针
 * @param i_a A相电流
 * @param i_b B相电流
 * @param i_c C相电流
 * @return 故障码（如有）
 */
static inline uint32_t Detection_CheckCurrent(MOTOR_DATA *m, float i_a,
                                              float i_b, float i_c) {
  (void)m; // 参数保留用于扩展
  uint32_t fault = FAULT_NONE;

  if (!s_config.enable_current_protection)
    return fault;

  // 记录三相电流峰值
  float i_mag = fmaxf(fabsf(i_a), fmaxf(fabsf(i_b), fabsf(i_c)));
  s_state.current_peak = i_mag;

  // 分别检测各相过流
  if (fabsf(i_a) > s_config.over_current_threshold) {
    fault |= FAULT_CURRENT_A;
  }
  if (fabsf(i_b) > s_config.over_current_threshold) {
    fault |= FAULT_CURRENT_B;
  }
  if (fabsf(i_c) > s_config.over_current_threshold) {
    fault |= FAULT_CURRENT_C;
  }

  return fault;
}

/**
 * @brief 温度保护检测
 * @param m 电机数据指针
 * @param temp 当前温度
 * @return 故障码（如有）
 */
static inline uint32_t Detection_CheckTemperature(MOTOR_DATA *m, float temp) {
  (void)m; // 参数保留用于扩展
  uint32_t fault = FAULT_NONE;

  if (!s_config.enable_temp_protection)
    return fault;

  // 滤波
  s_state.temp_filtered = s_state.temp_filtered * FAULT_FILTER_ALPHA_SLOW +
                          temp * FAULT_FILTER_ALPHA_FAST;

  // 检测
  if (s_state.temp_filtered > s_config.over_temp_threshold)
    fault |= FAULT_OVER_TEMP;

  return fault;
}

/**
 * @brief 堵转保护检测
 * @param m 电机数据指针
 * @param i_a A相电流
 * @param i_b B相电流
 * @param i_c C相电流
 * @param velocity 电机速度
 * @return 故障码（如有）
 */
static inline uint32_t Detection_CheckStall(MOTOR_DATA *m, float i_a, float i_b,
                                            float i_c, float velocity) {
  (void)m; // 参数保留用于扩展
  uint32_t fault = FAULT_NONE;

  if (!s_config.enable_stall_protection)
    return fault;

  float i_rms = sqrtf((i_a * i_a + i_b * i_b + i_c * i_c) / 3.0f);

  if (i_rms > s_config.stall_current_threshold &&
      fabsf(velocity) < s_config.stall_velocity_threshold) {
    s_state.stall_counter++;
    // 控制频率计数转换
    if (s_state.stall_counter >
        s_config.stall_detect_time_ms * STALL_DETECT_COUNT_PER_MS) {
      fault |= FAULT_STALL_OVERLOAD;
      s_state.is_stall = true;
    }
  } else {
    s_state.stall_counter = 0;
    s_state.is_stall = false;
  }

  return fault;
}

/**
 * @brief CAN通信超时检测
 * @param m 电机数据指针
 * @return 故障码（如有）
 */
static inline uint32_t Detection_CheckCANTimeout(MOTOR_DATA *m) {
  (void)m; // 参数保留用于扩展

  if (s_config.enable_can_timeout && s_config.can_timeout_ms > 0) {
    if ((HAL_GetSystemTick() - s_state.last_can_time) >
        s_config.can_timeout_ms) {
      s_state.is_can_timeout = true;
      // 超时通常不直接触发Fault位，而是导致状态切换，视具体需求而定
    }
  }

  return FAULT_NONE;
}

/**
 * @brief 编码器通信/校验错误检测
 * @param m 电机数据指针
 * @return 故障码（如有）
 */
static inline uint32_t Detection_CheckEncoder(MOTOR_DATA *m) {
  if (m == NULL || m->components.encoder == NULL) {
    return FAULT_NONE;
  }

  MT6816_Handle_t *enc = ENC(m);
  if (enc->last_status != MT6816_OK) {
    if (s_state.encoder_err_consecutive < 0xFFFFFFFFu) {
      s_state.encoder_err_consecutive++;
    }
  } else {
    s_state.encoder_err_consecutive = 0;
  }

  s_state.encoder_err_count = enc->rx_err_count + enc->check_err_count;

  if (s_state.encoder_err_consecutive >= FAULT_ENCODER_ERR_CONSECUTIVE_MAX ||
      enc->rx_err_count >= FAULT_ENCODER_ERR_COUNT_MAX ||
      enc->check_err_count >= FAULT_ENCODER_ERR_COUNT_MAX) {
    return FAULT_ENCODER_LOSS;
  }

  return FAULT_NONE;
}

/**
 * @brief 综合故障检测
 * @param motor 电机数据指针
 * @return 故障码位掩码
 *
 * @note 执行流程：
 *   1. 获取传感器数据
 *   2. 电压保护检测（过压/欠压）
 *   3. 电流保护检测（过流）
 *   4. 温度保护检测（过温）
 *   5. 堵转保护检测（电流高+速度低）
 *   6. CAN通信超时检测
 */
uint32_t Detection_Check(void *motor) {
  if (motor == NULL)
    return FAULT_NONE;

  MOTOR_DATA *m = (MOTOR_DATA *)motor;
  uint32_t fault = FAULT_NONE;

  // 获取物理量 (新架构 algo_input)
  float vbus = m->algo_input.Vbus;
  float i_a = m->algo_input.Ia;
  float i_b = m->algo_input.Ib;
  float i_c = m->algo_input.Ic;
  float temp = HAL_GetTemperature();
  float velocity = ENC(m)->velocity_rad_s;

  // 分模块检测
  fault |= Detection_CheckVoltage(m, vbus);
  fault |= Detection_CheckCurrent(m, i_a, i_b, i_c);
  fault |= Detection_CheckTemperature(m, temp);
  fault |= Detection_CheckStall(m, i_a, i_b, i_c, velocity);
  fault |= Detection_CheckCANTimeout(m);

  return fault;
}

/**
 * @brief 快速故障检测 (20kHz)
 * @note 仅检测过流等致命故障，执行时间约1μs
 */
uint32_t Detection_Check_Fast(void *motor) {
  if (motor == NULL)
    return FAULT_NONE;

  MOTOR_DATA *m = (MOTOR_DATA *)motor;
  uint32_t fault = FAULT_NONE;

  // 仅检测电流（最紧急的故障）
  float i_a = m->algo_input.Ia;
  float i_b = m->algo_input.Ib;
  float i_c = m->algo_input.Ic;

  fault |= Detection_CheckCurrent(m, i_a, i_b, i_c);

  return fault;
}

/**
 * @brief 慢速故障检测 (200Hz)
 * @note 检测缓变故障，执行时间约3μs
 */
uint32_t Detection_Check_Slow(void *motor) {
  if (motor == NULL)
    return FAULT_NONE;

  MOTOR_DATA *m = (MOTOR_DATA *)motor;
  uint32_t fault = FAULT_NONE;

  // 获取物理量
  float vbus = m->algo_input.Vbus;
  float i_a = m->algo_input.Ia;
  float i_b = m->algo_input.Ib;
  float i_c = m->algo_input.Ic;
  float temp = HAL_GetTemperature();
  float velocity = ENC(m)->velocity_rad_s;

  // 检测慢速变化的故障
  fault |= Detection_CheckVoltage(m, vbus);
  fault |= Detection_CheckTemperature(m, temp);
  fault |= Detection_CheckStall(m, i_a, i_b, i_c, velocity);
  fault |= Detection_CheckCANTimeout(m);
  fault |= Detection_CheckEncoder(m);

  return fault;
}

const DetectionState *Detection_GetState(void) { return &s_state; }

void Detection_FeedWatchdog(uint32_t timestamp) {
  s_state.last_can_time = timestamp;
}

DetectionConfig *Detection_GetConfig(void) { return &s_config; }
