#include "fault_detection.h"
#include "hal_abstraction.h"
#include "hal_encoder.h"
#include "motor.h"
#include "config.h"
#ifdef BOARD_XSTAR
#include "board_config_xstar.h"
#include "hall_encoder.h"
#include "abz_encoder.h"
#else
#include "mt6816_encoder.h"
#endif
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
  (void)m; // param
  if (s_config.enable_can_timeout && s_config.can_timeout_ms > 0) {
    if ((HAL_GetSystemTick() - s_state.last_can_time) >
        s_config.can_timeout_ms) {
      s_state.is_can_timeout = true;
      // timeoutFault，state，
    }
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
#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  if (!hall_data.signal_valid || hall_data.hall_state == 0u || hall_data.hall_state == 7u) {
    if (s_state.encoder_err_consecutive < 0xFFFFFFFFu) {
      s_state.encoder_err_consecutive++;
    }
  } else {
    s_state.encoder_err_consecutive = 0;
  }
  s_state.encoder_err_count = s_state.encoder_err_consecutive;
  if (s_state.encoder_err_consecutive >= FAULT_ENCODER_ERR_CONSECUTIVE_MAX) {
    return FAULT_ENCODER_LOSS;
  }
  return FAULT_NONE;
#else
  (void)m;
  s_state.encoder_err_consecutive = 0;
  s_state.encoder_err_count = 0;
  return FAULT_NONE;
#endif
#else
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
