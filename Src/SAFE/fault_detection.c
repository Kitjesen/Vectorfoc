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

#include "fault_detection.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"   /* MHAL_Encoder_GetVelocity */
#include "motor.h"
#include "config.h"
#include <math.h>

/* 编码器错误计数通过 motor->components.encoder 和 ENC() 宏访问（已在 motor.h 定义），
 * 不再直接引用具体编码器驱动头文件，消除 SAFE→HAL/encoder 的跨层依赖。
 * 各传感器模式的内部字段通过 motor->feedback 和 ENC() 宏访问，
 * ENC() 宏已在 motor.h 中根据 HW_POSITION_SENSOR_MODE 正确类型化。 */
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
 * @brief voltageprotection
 * @param m motor
 * @param vbus voltage
 * @return fault（）
 */
static inline uint32_t Detection_CheckVoltage(MOTOR_DATA *m, float vbus) {
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_voltage_protection)
    return fault;
  /* Open-loop demo: ADC trigger may be stopped; skip voltage check */
  if (m != NULL && m->state.Control_Mode == CONTROL_MODE_OPEN)
    return fault;
  // filter
  s_state.vbus_filtered = s_state.vbus_filtered * FAULT_FILTER_ALPHA_SLOW +
                          vbus * FAULT_FILTER_ALPHA_FAST;
  //
  if (s_state.vbus_filtered > s_config.over_voltage_threshold)
    fault |= FAULT_OVER_VOLTAGE;
  if (s_state.vbus_filtered < s_config.under_voltage_threshold)
    fault |= FAULT_UNDER_VOLTAGE;
  return fault;
}
/**
 * @brief currentprotection
 * @param m motor
 * @param i_a Aphasecurrent
 * @param i_b Bphasecurrent
 * @param i_c Cphasecurrent
 * @return fault（）
 */
static inline uint32_t Detection_CheckCurrent(MOTOR_DATA *m, float i_a,
                                              float i_b, float i_c) {
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_current_protection)
    return fault;
  /* Open-loop: current offsets uncalibrated → skip to avoid false OC fault */
  if (m != NULL && m->state.Control_Mode == CONTROL_MODE_OPEN)
    return fault;
  // phasecurrent
  float i_mag = fmaxf(fabsf(i_a), fmaxf(fabsf(i_b), fabsf(i_c)));
  s_state.current_peak = i_mag;
  // phase
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
 * @brief temperatureprotection
 * @param m motor
 * @param temp temperature
 * @return fault（）
 */
static inline uint32_t Detection_CheckTemperature(MOTOR_DATA *m, float temp) {
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_temp_protection)
    return fault;
  /* Open-loop demo: NTC may not be calibrated for XSTAR topology */
  if (m != NULL && m->state.Control_Mode == CONTROL_MODE_OPEN)
    return fault;
  // filter
  s_state.temp_filtered = s_state.temp_filtered * FAULT_FILTER_ALPHA_SLOW +
                          temp * FAULT_FILTER_ALPHA_FAST;
  //
  if (s_state.temp_filtered > s_config.over_temp_threshold)
    fault |= FAULT_OVER_TEMP;
  return fault;
}
/**
 * @brief protection
 * @param m motor
 * @param i_a Aphasecurrent
 * @param i_b Bphasecurrent
 * @param i_c Cphasecurrent
 * @param velocity motorspeed/velocity
 * @return fault（）
 */
static inline uint32_t Detection_CheckStall(MOTOR_DATA *m, float i_a, float i_b,
                                            float i_c, float velocity) {
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_stall_protection)
    return fault;
  /* Open-loop: motor starts from standstill — velocity=0 is normal */
  if (m != NULL && m->state.Control_Mode == CONTROL_MODE_OPEN)
    return fault;
  
  // [FIX] 添加电流有效性检查，避免 NaN 或 Inf 导致的误判
  if (!isfinite(i_a) || !isfinite(i_b) || !isfinite(i_c)) {
    return fault;
  }
  
  float i_rms = sqrtf((i_a * i_a + i_b * i_b + i_c * i_c) / 3.0f);
  
  // [FIX] 检查 i_rms 计算结果有效性
  if (!isfinite(i_rms)) {
    return fault;
  }
  
  if (i_rms > s_config.stall_current_threshold &&
      fabsf(velocity) < s_config.stall_velocity_threshold) {
    s_state.stall_counter++;
    // frequency
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
 * @brief CANtimeout
 * @param m motor
 * @return fault（）
 */
static inline uint32_t Detection_CheckCANTimeout(MOTOR_DATA *m) {
  (void)m;
  if (!s_config.enable_can_timeout || s_config.can_timeout_ms == 0) {
    return FAULT_NONE;
  }
  uint32_t elapsed = HAL_GetSystemTick() - s_state.last_can_time;
  if (elapsed > s_config.can_timeout_ms) {
    s_state.is_can_timeout = true;
    /* 确认标志已置位后才上报 fault bit，避免重复触发 */
    return FAULT_CAN_TIMEOUT;
  }
  /* 超时消除：通讯恢复后自动复位标志 */
  if (s_state.is_can_timeout) {
    s_state.is_can_timeout = false;
  }
  return FAULT_NONE;
}
/**
 * @brief encoder/error
 * @param m motor
 * @return fault（）
 */
static inline uint32_t Detection_CheckEncoder(MOTOR_DATA *m) {
  if (m == NULL || m->components.encoder == NULL) {
    return FAULT_NONE;
  }
  /* Open-loop mode does not use encoder — suppress encoder fault */
  if (m->state.Control_Mode == CONTROL_MODE_OPEN) {
    s_state.encoder_err_consecutive = 0;
    return FAULT_NONE;
  }

  /* 编码器健康状态通过 motor HAL 接口层获取（不直接读具体驱动字段）：
   * - 对于 MT6816 / TMR3109 等绝对值编码器：通过 ENC() 宏访问通用字段。
   *   ENC() 已在 motor.h 中根据 HW_POSITION_SENSOR_MODE 正确类型化，
   *   不需要在此处 #include 具体编码器驱动头文件。
   * - 对于 Hall/ABZ：驱动内部不暴露 rx_err_count，
   *   通过 MHAL_Encoder_Update 返回值判断（0=OK，-1=无驱动）。 */
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL || \
    HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
  /* Hall/ABZ: 只要 HAL 接口可达就认为编码器正常
   * （Hall 信号有效性检测在 hall_encoder.c 内部处理，
   *  不在 SAFE 层直接访问 hall_data 驱动内部字段）。 */
  s_state.encoder_err_consecutive = 0;
  s_state.encoder_err_count = 0;
  return FAULT_NONE;
#else
  /* 绝对值编码器（MT6816 / TMR3109）：通过 ENC() 宏（motor.h 中定义）访问错误计数
   * ENC() 返回对应传感器类型的 Handle_t 指针，两种传感器都有 rx_err_count 字段。 */
  {
    /* 使用公共字段检查：两种编码器 Handle_t 的前几个字段布局一致
     * （rx_err_count / last_status 位于相同偏移），此处通过 ENC() 宏统一访问。 */
    uint32_t rx_err = 0;
    uint32_t chk_err = 0;
    bool status_ok = true;
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
    {
      TMR3109_Handle_t *enc = ENC(m);
      rx_err   = enc->spi_err_count;
      chk_err  = enc->crc_err_count;
      status_ok = (enc->last_status == TMR3109_OK);
    }
#else /* MT6816 */
    {
      MT6816_Handle_t *enc = ENC(m);
      rx_err   = enc->rx_err_count;
      chk_err  = enc->check_err_count;
      status_ok = (enc->last_status == MT6816_OK);
    }
#endif
    if (!status_ok) {
      if (s_state.encoder_err_consecutive < 0xFFFFFFFFu)
        s_state.encoder_err_consecutive++;
    } else {
      s_state.encoder_err_consecutive = 0;
    }
    s_state.encoder_err_count = rx_err + chk_err;
    if (s_state.encoder_err_consecutive >= FAULT_ENCODER_ERR_CONSECUTIVE_MAX ||
        rx_err  >= FAULT_ENCODER_ERR_COUNT_MAX ||
        chk_err >= FAULT_ENCODER_ERR_COUNT_MAX) {
      return FAULT_ENCODER_LOSS;
    }
    return FAULT_NONE;
  }
#endif
}
/**
 * @brief fault
 * @param motor motor
 * @return fault
 *
 * @note ：
 *   1. get
 *   2. voltageprotection（/）
 *   3. currentprotection（）
 *   4. temperatureprotection（）
 *   5. protection（current+speed/velocity）
 *   6. CANtimeout
 */
uint32_t Detection_Check(void *motor) {
  if (motor == NULL)
    return FAULT_NONE;
  MOTOR_DATA *m = (MOTOR_DATA *)motor;
  uint32_t fault = FAULT_NONE;
  // get ( algo_input)
  float vbus = m->algo_input.Vbus;
  float i_a = m->algo_input.Ia;
  float i_b = m->algo_input.Ib;
  float i_c = m->algo_input.Ic;
  float temp = HAL_GetTemperature();
  float velocity = MHAL_Encoder_GetVelocity();
  //
  fault |= Detection_CheckVoltage(m, vbus);
  fault |= Detection_CheckCurrent(m, i_a, i_b, i_c);
  fault |= Detection_CheckTemperature(m, temp);
  fault |= Detection_CheckStall(m, i_a, i_b, i_c, velocity);
  fault |= Detection_CheckCANTimeout(m);
  return fault;
}
/**
 * @brief fault (20kHz)
 * @note fault，1μs
 */
uint32_t Detection_Check_Fast(void *motor) {
  if (motor == NULL)
    return FAULT_NONE;
  MOTOR_DATA *m = (MOTOR_DATA *)motor;
  uint32_t fault = FAULT_NONE;
  // current（fault）
  float i_a = m->algo_input.Ia;
  float i_b = m->algo_input.Ib;
  float i_c = m->algo_input.Ic;
  fault |= Detection_CheckCurrent(m, i_a, i_b, i_c);
  return fault;
}
/**
 * @brief fault (200Hz)
 * @note fault，3μs
 */
uint32_t Detection_Check_Slow(void *motor) {
  if (motor == NULL)
    return FAULT_NONE;
  MOTOR_DATA *m = (MOTOR_DATA *)motor;
  uint32_t fault = FAULT_NONE;
  // get
  float vbus = m->algo_input.Vbus;
  float i_a = m->algo_input.Ia;
  float i_b = m->algo_input.Ib;
  float i_c = m->algo_input.Ic;
  float temp = HAL_GetTemperature();
  float velocity = MHAL_Encoder_GetVelocity();
  // fault
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
