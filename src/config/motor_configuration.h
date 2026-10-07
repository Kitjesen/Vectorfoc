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
 * @file motor_configuration.h
 * @brief Motor parameters, timing, calibration and protection limits.
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#include "common.h"
/* ==============================================================================
   1. 硬件参数
   固件构建：从 board_configuration.h 推导（依赖 STM32 HAL 头文件，不适合 host 测试）
   测试构建：使用 VectorFOC G431 默认值（TEST_ENV 由 tests/CMakeLists.txt 定义）
   ==============================================================================
 */
#ifndef TEST_ENV
#include "board_configuration.h"  // HW_SHUNT_RESISTANCE, HW_OPAMP_GAIN, HW_FAC_CURRENT …
#define PWM_FREQUENCY           HW_PWM_FREQ_HZ
#ifndef MCPWM_DEADTIME_CLOCKS
#define MCPWM_DEADTIME_CLOCKS   HW_PWM_DEADTIME_CLKS
#endif
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
#define PWM_FREQUENCY           20000u
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
#define PWM_ARR                 (SYS_CLOCK_HZ / (2u * PWM_FREQUENCY))
#define DEADTIME_COMP           MCPWM_DEADTIME_CLOCKS
/* ============================================================================
 * 2. Control timing
 * One ADC conversion completes per PWM cycle. Every fast-path time step and
 * slower schedule is derived from that same integer base frequency.
 * ========================================================================== */
#define CURRENT_SAMPLE_FREQUENCY_HZ PWM_FREQUENCY
#define FOC_LOOP_FREQUENCY_HZ CURRENT_SAMPLE_FREQUENCY_HZ
#define COMMAND_RAMP_FREQUENCY_HZ FOC_LOOP_FREQUENCY_HZ
#define CURRENT_SAMPLE_PERIOD_S (1.0f / (float)CURRENT_SAMPLE_FREQUENCY_HZ)
#define FOC_LOOP_PERIOD_S CURRENT_SAMPLE_PERIOD_S
#define COMMAND_RAMP_PERIOD_S FOC_LOOP_PERIOD_S

#ifndef VELOCITY_LOOP_FREQUENCY_HZ
#define VELOCITY_LOOP_FREQUENCY_HZ 5000u
#endif
#ifndef POSITION_LOOP_FREQUENCY_HZ
#define POSITION_LOOP_FREQUENCY_HZ 1000u
#endif
#ifndef STATE_MACHINE_UPDATE_FREQUENCY_HZ
#define STATE_MACHINE_UPDATE_FREQUENCY_HZ 1000u
#endif
#ifndef TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ
#define TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ 5000u
#endif
#ifndef SCOPE_LOG_FREQUENCY_HZ
#define SCOPE_LOG_FREQUENCY_HZ 1000u
#endif
#ifndef SCOPE_LOG_ENABLED
#define SCOPE_LOG_ENABLED 1u
#endif

/* Reject truncation: the declared controller period must match its schedule. */
#if FOC_LOOP_FREQUENCY_HZ <= 0
#error "FOC_LOOP_FREQUENCY_HZ must be positive"
#endif
#if VELOCITY_LOOP_FREQUENCY_HZ <= 0 || VELOCITY_LOOP_FREQUENCY_HZ > FOC_LOOP_FREQUENCY_HZ
#error "VELOCITY_LOOP_FREQUENCY_HZ must be between 1 and FOC_LOOP_FREQUENCY_HZ"
#elif FOC_LOOP_FREQUENCY_HZ % VELOCITY_LOOP_FREQUENCY_HZ != 0
#error "VELOCITY_LOOP_FREQUENCY_HZ must divide FOC_LOOP_FREQUENCY_HZ exactly"
#endif
#if POSITION_LOOP_FREQUENCY_HZ <= 0 || POSITION_LOOP_FREQUENCY_HZ > FOC_LOOP_FREQUENCY_HZ
#error "POSITION_LOOP_FREQUENCY_HZ must be between 1 and FOC_LOOP_FREQUENCY_HZ"
#elif FOC_LOOP_FREQUENCY_HZ % POSITION_LOOP_FREQUENCY_HZ != 0
#error "POSITION_LOOP_FREQUENCY_HZ must divide FOC_LOOP_FREQUENCY_HZ exactly"
#endif
#if STATE_MACHINE_UPDATE_FREQUENCY_HZ <= 0 || STATE_MACHINE_UPDATE_FREQUENCY_HZ > FOC_LOOP_FREQUENCY_HZ
#error "STATE_MACHINE_UPDATE_FREQUENCY_HZ must be between 1 and FOC_LOOP_FREQUENCY_HZ"
#elif FOC_LOOP_FREQUENCY_HZ % STATE_MACHINE_UPDATE_FREQUENCY_HZ != 0
#error "STATE_MACHINE_UPDATE_FREQUENCY_HZ must divide FOC_LOOP_FREQUENCY_HZ exactly"
#endif
#if TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ <= 0 || TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ > FOC_LOOP_FREQUENCY_HZ
#error "TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ must be between 1 and FOC_LOOP_FREQUENCY_HZ"
#elif FOC_LOOP_FREQUENCY_HZ % TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ != 0
#error "TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ must divide FOC_LOOP_FREQUENCY_HZ exactly"
#endif
#if SCOPE_LOG_FREQUENCY_HZ <= 0 || SCOPE_LOG_FREQUENCY_HZ > FOC_LOOP_FREQUENCY_HZ
#error "SCOPE_LOG_FREQUENCY_HZ must be between 1 and FOC_LOOP_FREQUENCY_HZ"
#elif FOC_LOOP_FREQUENCY_HZ % SCOPE_LOG_FREQUENCY_HZ != 0
#error "SCOPE_LOG_FREQUENCY_HZ must divide FOC_LOOP_FREQUENCY_HZ exactly"
#endif

#define VELOCITY_LOOP_PERIOD_S (1.0f / (float)VELOCITY_LOOP_FREQUENCY_HZ)
#define POSITION_LOOP_PERIOD_S (1.0f / (float)POSITION_LOOP_FREQUENCY_HZ)
#define TORQUE_COMPENSATION_UPDATE_PERIOD_S \
  (1.0f / (float)TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ)
#define VELOCITY_LOOP_DECIMATION (FOC_LOOP_FREQUENCY_HZ / VELOCITY_LOOP_FREQUENCY_HZ)
#define POSITION_LOOP_DECIMATION (FOC_LOOP_FREQUENCY_HZ / POSITION_LOOP_FREQUENCY_HZ)
#define STATE_MACHINE_DECIMATION \
  (FOC_LOOP_FREQUENCY_HZ / STATE_MACHINE_UPDATE_FREQUENCY_HZ)
#define TORQUE_COMPENSATION_UPDATE_DECIMATION \
  (FOC_LOOP_FREQUENCY_HZ / TORQUE_COMPENSATION_UPDATE_FREQUENCY_HZ)
#define SCOPE_LOG_DECIMATION (FOC_LOOP_FREQUENCY_HZ / SCOPE_LOG_FREQUENCY_HZ)
#if VELOCITY_LOOP_DECIMATION > 65535u || POSITION_LOOP_DECIMATION > 65535u || \
    STATE_MACHINE_DECIMATION > 65535u || TORQUE_COMPENSATION_UPDATE_DECIMATION > 65535u || \
    SCOPE_LOG_DECIMATION > 65535u
#error "Control decimation must fit the 16-bit scheduling counters"
#endif
#if SYS_CLOCK_HZ % (2u * PWM_FREQUENCY) != 0 || PWM_ARR == 0 || PWM_ARR > 65535u
#error "PWM frequency must divide the center-aligned timer clock exactly"
#endif

/* ==============================================================================
   3.  (Calibration Configuration)
   ==============================================================================
 */
/* calibrationlimit */
#define VOLTAGE_MAX_CALIB 2.0f // calibrationvoltage [V]
#define CURRENT_MAX_CALIB 5.0f // calibrationcurrent [A]
/* calibration (period) */
#define CURRENT_CALIB_CYCLES CURRENT_SAMPLE_FREQUENCY_HZ // Zero-offset averaging: 1 s
#define RS_CALIB_CYCLES CURRENT_SAMPLE_FREQUENCY_HZ      // Resistance calibration: 1 s
#define LS_CALIB_CYCLES (CURRENT_SAMPLE_FREQUENCY_HZ / 4u) // Inductance calibration: 0.25 s
#define FLUX_CALIB_DURATION 2.0f   // fluxcalibration: 2.0s
#define FLUX_CALIB_CYCLES ((uint32_t)(FLUX_CALIB_DURATION * CURRENT_SAMPLE_FREQUENCY_HZ))
/* calibrationparam */
#define CALIB_PHASE_VEL (2.0f * M_PI) // encodercalibrationspeed/velocity [rad/s]
#define FLUX_CALIB_VEL (5.0f * M_PI)  // fluxcalibrationspeed/velocity [rad/s]
#define CALIB_FILTER_ALPHA 0.01f      // calibrationfilter
/* param */
#define MAX_POLE_PAIRS 20          // maximum supported calibration pole pairs
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
/* filter - : faultfilter fault_definition.h */
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
#define DEFAULT_CAN_BAUDRATE 0   // 0=1Mbps, 1=500kbps, 2=250kbps
#define DEFAULT_PROTOCOL_TYPE 0  // Inovxio
#define DEFAULT_CAN_TIMEOUT_MS 1000 // command watchdog [ms]; 0 disables it
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
/* : faultthreshold safety/fault_definition.h */
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
