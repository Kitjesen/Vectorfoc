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
#include "hal_encoder.h"
#include "motor.h"
#include "position_sensor.h"
#include <math.h>
static DetectionConfig s_config;
static DetectionState s_state = {0};
static bool s_can_timeout_armed = false;
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
  s_state.stall_start_time_ms = 0;
  s_state.stall_tracking = false;
  s_state.is_stall = false;
  s_state.is_can_timeout = false;
  s_state.vbus_filtered = 0.0f;
  s_state.temp_filtered = 25.0f;
  s_state.encoder_err_consecutive = 0;
  s_state.encoder_err_count = 0;
  s_state.vbus_initialized = false;
  s_state.temp_initialized = false;
  s_state.last_can_time = HAL_GetSystemTick();
  s_can_timeout_armed = false;
}
/**
 * @brief voltageprotection
 * @param m motor
 * @param vbus voltage
 * @return fault（）
 */
static inline uint32_t Detection_CheckVoltage(MOTOR_DATA *m, float vbus) {
  (void)m;
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_voltage_protection)
    return fault;
  if (!isfinite(vbus))
    return FAULT_OVER_VOLTAGE | FAULT_UNDER_VOLTAGE;
  // filter
  if (!s_state.vbus_initialized) {
    s_state.vbus_filtered = vbus;
    s_state.vbus_initialized = true;
  } else {
    s_state.vbus_filtered = s_state.vbus_filtered * FAULT_FILTER_ALPHA_SLOW +
                            vbus * FAULT_FILTER_ALPHA_FAST;
  }
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
  (void)m;
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_current_protection)
    return fault;
  /* Open-loop: current offsets uncalibrated → skip to avoid false OC fault */
  if (!isfinite(i_a)) fault |= FAULT_CURRENT_A;
  if (!isfinite(i_b)) fault |= FAULT_CURRENT_B;
  if (!isfinite(i_c)) fault |= FAULT_CURRENT_C;
  if (fault != FAULT_NONE) return fault;
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
  (void)m;
  uint32_t fault = FAULT_NONE;
  if (!s_config.enable_temp_protection)
    return fault;
  /* Open-loop demo: NTC may not be calibrated for XSTAR topology */
  if (!isfinite(temp))
    return FAULT_OVER_TEMP;
  // filter
  if (!s_state.temp_initialized) {
    s_state.temp_filtered = temp;
    s_state.temp_initialized = true;
  } else {
    s_state.temp_filtered = s_state.temp_filtered * FAULT_FILTER_ALPHA_SLOW +
                            temp * FAULT_FILTER_ALPHA_FAST;
  }
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
    uint32_t now = HAL_GetSystemTick();
    if (!s_state.stall_tracking) {
      s_state.stall_tracking = true;
      s_state.stall_start_time_ms = now;
    }
    s_state.stall_counter = now - s_state.stall_start_time_ms;
    if (s_state.stall_counter >= s_config.stall_detect_time_ms) {
      fault |= FAULT_STALL_OVERLOAD;
      s_state.is_stall = true;
    }
  } else {
    s_state.stall_counter = 0;
    s_state.stall_start_time_ms = 0;
    s_state.stall_tracking = false;
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
  if (!s_config.enable_can_timeout || s_config.can_timeout_ms == 0 ||
      !s_can_timeout_armed || m == NULL ||
      m->state.State_Mode != STATE_MODE_RUNNING) {
    s_state.is_can_timeout = false;
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
  PositionSensorHealth_t health;
  PositionSensorStatus_t status;

  if (m == NULL) {
    return FAULT_NONE;
  }
  /* Open-loop mode does not use encoder — suppress encoder fault */
  if (m->state.Control_Mode == CONTROL_MODE_OPEN) {
    s_state.encoder_err_consecutive = 0;
    return FAULT_NONE;
  }

  /* PositionSensor already counts failed real-time updates.  Consume its
   * snapshot directly so the 200 Hz safety poll cannot count the same failed
   * frame again.  total_failures and the driver-specific transport score can
   * overlap, therefore use the larger value rather than summing them. */
  status = (PositionSensor_GetDescriptor() != NULL &&
            PositionSensor_IsInitialized())
               ? PositionSensor_GetHealth(&health)
               : POSITION_SENSOR_STATUS_NOT_INITIALIZED;
  if (status == POSITION_SENSOR_STATUS_OK) {
    uint32_t reported_total =
        health.total_failures > health.transport_error_score
            ? health.total_failures
            : health.transport_error_score;
    if (health.valid) {
      s_state.encoder_err_consecutive = 0u;
    } else if (health.consecutive_failures > 0u) {
      s_state.encoder_err_consecutive = health.consecutive_failures;
    } else {
      /* A commutation/signal-validity failure (notably Hall) may have no
       * transport counter. In that case each slow health sample is the only
       * observable failure event, so retain the legacy debounce locally. */
      if (s_state.encoder_err_consecutive < UINT32_MAX) {
        s_state.encoder_err_consecutive++;
      }
      if (s_state.encoder_err_count < UINT32_MAX) {
        s_state.encoder_err_count++;
      }
    }
    if (reported_total > s_state.encoder_err_count) {
      s_state.encoder_err_count = reported_total;
    }
  } else {
    /* A missing/uninitialized adapter has no module-owned counter to consume.
     * Count one unavailable health sample per slow safety cycle, preserving the
     * existing consecutive threshold and recovery behavior. */
    if (s_state.encoder_err_consecutive < 0xFFFFFFFFu) {
      s_state.encoder_err_consecutive++;
    }
    if (s_state.encoder_err_count < 0xFFFFFFFFu) {
      s_state.encoder_err_count++;
    }
  }

  if (s_state.encoder_err_consecutive >= FAULT_ENCODER_ERR_CONSECUTIVE_MAX) {
    return FAULT_ENCODER_LOSS;
  }
  return FAULT_NONE;
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
  float temp = m->feedback.temperature;
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
  float temp = m->feedback.temperature;
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
  if (s_config.enable_can_timeout && s_config.can_timeout_ms > 0U) {
    s_can_timeout_armed = true;
  }
}
void Detection_SetCANTimeout(uint32_t timeout_ms) {
  uint32_t now_ms = HAL_GetSystemTick();
  uint32_t critical_state = HAL_EnterCritical();
  s_config.enable_can_timeout = false;
  s_config.can_timeout_ms = timeout_ms;
  s_can_timeout_armed = false;
  s_state.is_can_timeout = false;
  s_state.last_can_time = now_ms;
  s_config.enable_can_timeout = timeout_ms > 0U;
  HAL_ExitCritical(critical_state);
}
DetectionConfig *Detection_GetConfig(void) { return &s_config; }
