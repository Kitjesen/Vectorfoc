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

/**
 * @file config.h
 * @brief motorconfig
 * @note config、、param、protectionthreshold
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#include "common.h"
/* ==============================================================================
   1. 硬件参数
   固件构建：从 board_config.h 推导（依赖 STM32 HAL 头文件，不适合 host 测试）
   测试构建：使用 VectorFOC G431 默认值（TEST_ENV 由 test/CMakeLists.txt 定义）
   ==============================================================================
 */
#ifndef TEST_ENV
#include "board_config.h"  // HW_SHUNT_RESISTANCE, HW_OPAMP_GAIN, HW_FAC_CURRENT …
#define SYS_CLOCK_HZ_F          ((float)SYS_CLOCK_HZ)
#define TIMER1_CLK_MHz          (SYS_CLOCK_HZ / 1000000UL)
#define PWM_FREQUENCY           HW_PWM_FREQ_HZ
#define MCPWM_DEADTIME_CLOCKS   HW_PWM_DEADTIME_CLKS
#define V_REG                   HW_ADC_MIDPOINT
#define FAC_CURRENT             HW_FAC_CURRENT
#define VOLTAGE_TO_ADC_FACTOR   HW_VOLTAGE_FACTOR
#define CURRENT_SHUNT_RES       HW_SHUNT_RESISTANCE
#define CURRENT_AMP_GAIN        HW_OPAMP_GAIN
#define VIN_R1                  HW_VBUS_R_LOW
#define VIN_R2                  HW_VBUS_R_HIGH
#else
/* host 测试环境：使用 VectorFOC G431 固定值，不依赖 HAL 头文件 */
#ifndef SYS_CLOCK_HZ
#define SYS_CLOCK_HZ            168000000UL
#endif
#define SYS_CLOCK_HZ_F          ((float)SYS_CLOCK_HZ)
#define TIMER1_CLK_MHz          (SYS_CLOCK_HZ / 1000000UL)
#define PWM_FREQUENCY           20000
#define MCPWM_DEADTIME_CLOCKS   20
#define V_REG                   1.65f
#define CURRENT_SHUNT_RES       0.02f
#define CURRENT_AMP_GAIN        50.0f
#define VIN_R1                  1000.0f
#define VIN_R2                  10000.0f
#define FAC_CURRENT             ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
#define VOLTAGE_TO_ADC_FACTOR   (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f))
#endif /* TEST_ENV */

/* PWM 辅助宏（两种环境通用） */
#define PWM_FREQUENCY_HZ        ((float)PWM_FREQUENCY)
#define PWM_PERIOD_CYCLES       (uint16_t)((TIMER1_CLK_MHz * 1000000u / PWM_FREQUENCY) & 0xFFFE)
#define PWM_ARR                 (uint16_t)(PWM_PERIOD_CYCLES / 2u)
#define DEADTIME_COMP           MCPWM_DEADTIME_CLOCKS
/* ==============================================================================
   2.  (System Timing)
   ==============================================================================
 */
#define CONTROL_FREQ_HZ 20000 // FOCfrequency (PWMfrequency)
#define CONTROL_PERIOD_S (1.0f / CONTROL_FREQ_HZ)
#define CURRENT_MEASURE_HZ PWM_FREQUENCY_HZ // currentsamplefrequency
#define CURRENT_MEASURE_PERIOD (1.0f / CURRENT_MEASURE_HZ)
// speed/velocity/positionupdatefrequency [Hz]
// current (20kHz)，CPU ( 5000Hz)
#define VEL_POS_HZ 20000
#define VEL_POS_PERIOD (1.0f / (float)VEL_POS_HZ)
/* ==============================================================================
   2.1 / (Decimation)
   ==============================================================================
 */
// runningfrequencytarget（<= CONTROL_FREQ_HZ）。set CONTROL_FREQ_HZ 。
#define FSM_UPDATE_HZ 1000  // DS402 stateupdatefrequency [Hz]
#define ADV_CONTROL_HZ 5000 // （feedforward//）updatefrequency [Hz]
#define HS_LOG_HZ 1000      // /updatefrequency [Hz]
#define HS_LOG_ENABLE 1     // 1: enable ISR , 0:
// safety（1）
#define MOTOR_DECIM_MIN1(x) ((x) < 1 ? 1 : (x))
#define MOTOR_DECIM_DIV(control_hz, target_hz)                                 \
  MOTOR_DECIM_MIN1(((target_hz) > 0) ? ((control_hz) / (target_hz)) : 1)
/* ==============================================================================
   3.  (Calibration Configuration)
   ==============================================================================
 */
/* calibrationlimit */
#define VOLTAGE_MAX_CALIB 2.0f // calibrationvoltage [V]
#define CURRENT_MAX_CALIB 5.0f // calibrationcurrent [A]
/* calibration (period) */
#define CURRENT_CALIB_CYCLES 20000 // currentcalibration: 1.0s
#define RS_CALIB_CYCLES 20000      // calibration: 1.0s
#define LS_CALIB_CYCLES 5000       // calibration: 0.25s
#define FLUX_CALIB_DURATION 2.0f   // fluxcalibration: 2.0s
#define FLUX_CALIB_CYCLES ((uint32_t)(FLUX_CALIB_DURATION * CURRENT_MEASURE_HZ))
/* calibrationparam */
#define CALIB_PHASE_VEL (2.0f * M_PI) // encodercalibrationspeed/velocity [rad/s]
#define FLUX_CALIB_VEL (5.0f * M_PI)  // fluxcalibrationspeed/velocity [rad/s]
#define CALIB_FILTER_ALPHA 0.01f      // calibrationfilter
/* param */
#define MAX_POLE_PAIRS 7          // pole pairs (≥ DEFAULT_POLE_PAIRS=7, reduces calib RAM)
#define SAMPLES_PER_POLE_PAIR 100 // sample
#define FLUX_VALID_MIN 0.001f     // flux [Wb]
#define FLUX_VALID_MAX 0.500f     // flux [Wb]
/* ==============================================================================
   4.  (Control Loop Configuration)
   ==============================================================================
 */
/* VEL_LIMIT_DEFAULT 已由 DEFAULT_VEL_LIMIT 替代，保留别名兼容旧引用 */
#define VEL_LIMIT_DEFAULT DEFAULT_VEL_LIMIT
/* PID output */
#define CURRENT_PID_MAX_OUT 58.0f // currentoutputvoltage [V] (14S)
#define VEL_PID_MAX_OUT 80.0f     // speed/velocityoutputcurrent [A]
#define POS_PID_MAX_OUT 50.0f     // positionoutputspeed/velocity [turn/s]
/* filter - : faultfilter fault_def.h */
/*  */
#define VELOCITY_ACCEL_MULTIPLIER 2.0f  // speed/velocityspeed/velocity
#define OPEN_MODE_DEFAULT_VELOCITY 5.0f // open loopmodespeed/velocity
#define OPEN_MODE_FIXED_VOLTAGE 3.0f    // open loopmodevoltage
/* MIT mode */
#define MIT_MODE_DECAY_FACTOR 0.95f
#define MIT_POSITION_STABILITY_THRESH (2.0f * M_PI)
#define MIT_VELOCITY_STABILITY_THRESH (20.0f * M_PI)
#define MIT_POSITION_ERROR_TOLERANCE 1.0f
/* Report (Telemetry) Filter */
#define REPORT_CURRENT_FILTER_FC 5.0f // Hz, for reported current only
/* V/F open-loop mode */
#define VF_BOOST_VOLTAGE    0.5f    // [V] voltage floor at zero speed (IR drop)
#define VF_BASE_VOLTAGE     3.0f    // [V] Vq magnitude at VF_BASE_VELOCITY
#define VF_BASE_VELOCITY   10.0f    // [turn/s] reference mechanical speed
/* I/F forced-current open-loop mode */
#define IF_DEFAULT_CURRENT  2.0f    // [A] default Iq when input_torque not set
/* Anti-cogging Calibration */
#define COGGING_MAP_SIZE 360
#define COGGING_CALIB_POS_THRESH_TURN 0.0005f
#define COGGING_CALIB_VEL_THRESH_TURN_S 0.001f
#define COGGING_CALIB_HOLD_CYCLES 400
#define COGGING_CALIB_VEL_LIMIT 5.0f
/* ==============================================================================
   5.  (Default Motor Parameters)
   ==============================================================================
 */
/* config */
#define DEFAULT_CAN_ID 0x01
#define DEFAULT_CAN_BAUDRATE 1   // 1Mbps
#define DEFAULT_PROTOCOL_TYPE 0  // Inovxio
#define DEFAULT_CAN_TIMEOUT_MS 0 // 0=Disable (Safety managed separately)
#define DEFAULT_ZERO_STA 0
#define DEFAULT_ADD_OFFSET 0.0f
#define DEFAULT_DAMPER_ENABLE 0
#define DEFAULT_RUN_MODE 2 // Velocity Control
/* param */
#define DEFAULT_RS 0.1f    // [Ohm]
#define DEFAULT_LS 0.001f  // [H]
#define DEFAULT_FLUX 0.01f // [Wb]
#define DEFAULT_POLE_PAIRS 7
#define DEFAULT_INERTIA 0.001f     // [kg*m^2]
#define DEFAULT_TORQUE_CONST 0.05f // [Nm/A]
/* limitparam */
#define DEFAULT_TORQUE_LIMIT 2.0f   // [Nm]
#define DEFAULT_VEL_LIMIT 1000.0f   // 速度限制 [turn/s]（控制层单位）
#define DEFAULT_VOLTAGE_LIMIT 58.0f // [V] (14S Battery Max)
#define DEFAULT_CURRENT_LIMIT 10.0f // [A]
/*  */
#define DEFAULT_TORQUE_RAMP_RATE 10.0f // [Nm/s]
#define DEFAULT_VEL_RAMP_RATE 100.0f   // [rpm/s]
#define DEFAULT_TRAJ_VEL 20.0f         // [turn/s]
#define DEFAULT_TRAJ_ACCEL 100.0f      // [turn/s^2]
#define DEFAULT_TRAJ_DECEL 100.0f      // [turn/s^2]
/* PID param (current) */
#define DEFAULT_CURRENT_P_GAIN 2.0f
#define DEFAULT_CURRENT_I_GAIN 100.0f
#define DEFAULT_CURRENT_BW 1000 // [Hz] or [rad/s]
#define DEFAULT_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define DEFAULT_PID_MAX_IOUT 10.0f
/* PID param (speed/velocity) */
#define DEFAULT_VEL_P_GAIN 0.05f
#define DEFAULT_VEL_I_GAIN 1.0f
#define DEFAULT_VEL_D_GAIN 0.0f
#define DEFAULT_VEL_MAX_OUT VEL_PID_MAX_OUT
#define DEFAULT_VEL_MAX_IOUT 10.0f
/* PID param (position) */
#define DEFAULT_POS_P_GAIN 20.0f
#define DEFAULT_POS_I_GAIN 0.0f
#define DEFAULT_POS_D_GAIN 0.0f
#define DEFAULT_POS_MAX_OUT POS_PID_MAX_OUT
#define DEFAULT_POS_MAX_IOUT 10.0f
/* MIT param */
#define DEFAULT_MIT_KP 0.0f
#define DEFAULT_MIT_KD 0.0f
/* LADRC speed/velocityparam */
#define DEFAULT_LADRC_ENABLE 0         /**< 0:  PID, 1:  LADRC */
#define DEFAULT_LADRC_OMEGA_O 300.0f   /**< observer [rad/s] ( 3~5 × omega_c) */
#define DEFAULT_LADRC_OMEGA_C 80.0f    /**<  [rad/s] ( 50~200) */
#define DEFAULT_LADRC_B0 100.0f        /**< gain b0 ≈ (3/2)*Pp*flux/J */
#define DEFAULT_LADRC_MAX_OUT VEL_PID_MAX_OUT /**< LADRC output [A] */
/* ==============================================================================
   6.  (Protection Thresholds)
   ==============================================================================
 */
#define DEFAULT_VBUS_VOLTAGE_V 52.0f // voltage [V] (14S Nominal)
#define VBUS_MIN_VALID_V 5.0f        // voltage [V]
/* : faultthreshold ALGO/motor/fault_def.h */
/* ==============================================================================
   6.  (Math Constants)
   ==============================================================================
 */
#ifndef M_PI
#define M_PI 3.14159265359f
#endif
#ifndef M_2PI
#define M_2PI (2.0f * M_PI)
#endif
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#endif // MOTOR_CONFIG_H
