/**
 * @file config.h
 * @brief 电机控制系统全局配置文件
 * @note 包含硬件配置、系统常量、控制参数、保护阈值等
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "common.h"

/* ==============================================================================
   1. 硬件与时钟配置 (Hardware Configuration)
   ==============================================================================
 */
// 注意: SYS_CLOCK_HZ 和硬件引脚映射在 Src/config/board_config.h 定义
// 这里提供浮点版本用于控制算法计算
#ifndef SYS_CLOCK_HZ
#define SYS_CLOCK_HZ 168000000UL  // 默认系统主频 168MHz
#endif
#define SYS_CLOCK_HZ_F ((float)SYS_CLOCK_HZ)
#define TIMER1_CLK_MHz (SYS_CLOCK_HZ / 1000000UL)

/* PWM 配置 */
#define PWM_FREQUENCY 20000 // PWM 开关频率 [Hz]
#define PWM_FREQUENCY_HZ ((float)PWM_FREQUENCY)

// PWM 周期计算: (TIMER_CLK / PWM_FREQ)
// 注意: 减去最后一位以确保偶数，用于中心对齐模式
#define PWM_PERIOD_CYCLES                                                      \
  (uint16_t)((TIMER1_CLK_MHz * 1000000u / PWM_FREQUENCY) & 0xFFFE)
#define PWM_ARR (uint16_t)(PWM_PERIOD_CYCLES / 2u)
#define MCPWM_DEADTIME_CLOCKS 20            // 死区时间 (时钟周期数)
#define DEADTIME_COMP MCPWM_DEADTIME_CLOCKS // 兼容性定义

/* ADC 采样配置 */
#define V_REG 1.65f             // Vref/2 (中点电压)
#define CURRENT_SHUNT_RES 0.02f // 采样电阻 [Ohm]
#define CURRENT_AMP_GAIN 50.0f  // 电流放大倍数
#define VIN_R1 1000.0f          // 电压采样分压电阻 R1
#define VIN_R2 10000.0f         // 电压采样分压电阻 R2

// ADC 转换系数
// 电流系数: 3.3V / 4096 / (R_shunt * Gain)
#define FAC_CURRENT ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
// 电压系数: ((R1+R2)/R1) * (3.3V / 4096)
#define VOLTAGE_TO_ADC_FACTOR (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f))

/* ==============================================================================
   2. 系统时序常量 (System Timing)
   ==============================================================================
 */
#define CONTROL_FREQ_HZ 20000 // FOC控制频率 (通常等于PWM频率)
#define CONTROL_PERIOD_S (1.0f / CONTROL_FREQ_HZ)

#define CURRENT_MEASURE_HZ PWM_FREQUENCY_HZ // 电流采样频率
#define CURRENT_MEASURE_PERIOD (1.0f / CURRENT_MEASURE_HZ)

// 速度/位置环更新频率 [Hz]
// 默认与电流环同频 (20kHz)，可根据CPU负载降低 (如 5000Hz)
#define VEL_POS_HZ 20000
#define VEL_POS_PERIOD (1.0f / (float)VEL_POS_HZ)

/* ==============================================================================
   2.1 控制回路降频/抽样 (Decimation)
   ==============================================================================
 */
// 运行频率目标（<= CONTROL_FREQ_HZ）。设置为 CONTROL_FREQ_HZ 表示不降频。
#define FSM_UPDATE_HZ 1000  // DS402 状态机更新频率 [Hz]
#define ADV_CONTROL_HZ 5000 // 高级控制（前馈/弱磁/齿槽补偿）更新频率 [Hz]
#define HS_LOG_HZ 1000      // 高速日志/示波更新频率 [Hz]
#define HS_LOG_ENABLE 1     // 1: 使能 ISR 内高速日志写入, 0: 关闭

// 编译期安全分频（结果最小为1）
#define MOTOR_DECIM_MIN1(x) ((x) < 1 ? 1 : (x))
#define MOTOR_DECIM_DIV(control_hz, target_hz)                                 \
  MOTOR_DECIM_MIN1(((target_hz) > 0) ? ((control_hz) / (target_hz)) : 1)

/* ==============================================================================
   3. 参数辨识配置 (Calibration Configuration)
   ==============================================================================
 */
/* 校准限制 */
#define VOLTAGE_MAX_CALIB 2.0f // 校准最大电压 [V]
#define CURRENT_MAX_CALIB 5.0f // 校准最大电流 [A]

/* 校准持续时间 (周期数) */
#define CURRENT_CALIB_CYCLES 20000 // 电流零偏校准: 1.0s
#define RS_CALIB_CYCLES 20000      // 电阻校准: 1.0s
#define LS_CALIB_CYCLES 5000       // 电感校准: 0.25s
#define FLUX_CALIB_DURATION 2.0f   // 磁链校准时长: 2.0s
#define FLUX_CALIB_CYCLES ((uint32_t)(FLUX_CALIB_DURATION * CURRENT_MEASURE_HZ))

/* 校准运动参数 */
#define CALIB_PHASE_VEL (2.0f * M_PI) // 编码器校准角速度 [rad/s]
#define FLUX_CALIB_VEL (5.0f * M_PI)  // 磁链校准角速度 [rad/s]
#define CALIB_FILTER_ALPHA 0.01f      // 校准滤波器系数

/* 参数验证范围 */
#define MAX_POLE_PAIRS 20         // 最大极对数
#define SAMPLES_PER_POLE_PAIR 100 // 每极对采样点数
#define FLUX_VALID_MIN 0.001f     // 最小有效磁链 [Wb]
#define FLUX_VALID_MAX 0.500f     // 最大有效磁链 [Wb]

/* ==============================================================================
   4. 控制环路配置 (Control Loop Configuration)
   ==============================================================================
 */
#define VEL_LIMIT_DEFAULT 1000.0f // 默认速度限制 [rpm]

/* PID 输出限幅 */
#define CURRENT_PID_MAX_OUT 58.0f // 电流环最大输出电压 [V] (14S电池)
#define VEL_PID_MAX_OUT 80.0f     // 速度环最大输出电流 [A]
#define POS_PID_MAX_OUT 50.0f     // 位置环最大输出速度 [turn/s]

/* 滤波器系数 - 注意: 故障检测滤波器已移至 fault_def.h */

/* 控制器常量 */
#define VELOCITY_ACCEL_MULTIPLIER 2.0f  // 速度加速度倍率
#define OPEN_MODE_DEFAULT_VELOCITY 5.0f // 开环模式默认速度
#define OPEN_MODE_FIXED_VOLTAGE 0.5f    // 开环模式固定电压

/* MIT 模式常量 */
#define MIT_MODE_DECAY_FACTOR 0.95f
#define MIT_POSITION_STABILITY_THRESH (2.0f * M_PI)
#define MIT_VELOCITY_STABILITY_THRESH (20.0f * M_PI)
#define MIT_POSITION_ERROR_TOLERANCE 1.0f

/* Report (Telemetry) Filter */
#define REPORT_CURRENT_FILTER_FC 5.0f // Hz, for reported current only

/* Anti-cogging Calibration */
#define COGGING_MAP_SIZE 360
#define COGGING_CALIB_POS_THRESH_TURN 0.0005f
#define COGGING_CALIB_VEL_THRESH_TURN_S 0.001f
#define COGGING_CALIB_HOLD_CYCLES 400
#define COGGING_CALIB_VEL_LIMIT 5.0f

/* ==============================================================================
   5. 默认电机参数 (Default Motor Parameters)
   ==============================================================================
 */
/* 通信配置 */
#define DEFAULT_CAN_ID 0x01
#define DEFAULT_CAN_BAUDRATE 1   // 1Mbps
#define DEFAULT_PROTOCOL_TYPE 1  // Inovxio
#define DEFAULT_CAN_TIMEOUT_MS 0 // 0=Disable (Safety managed separately)
#define DEFAULT_ZERO_STA 0
#define DEFAULT_ADD_OFFSET 0.0f
#define DEFAULT_DAMPER_ENABLE 0
#define DEFAULT_RUN_MODE 2 // Velocity Control

/* 物理参数 */
#define DEFAULT_RS 0.1f    // [Ohm]
#define DEFAULT_LS 0.001f  // [H]
#define DEFAULT_FLUX 0.01f // [Wb]
#define DEFAULT_POLE_PAIRS 7
#define DEFAULT_INERTIA 0.001f     // [kg*m^2]
#define DEFAULT_TORQUE_CONST 0.05f // [Nm/A]

/* 限制参数 */
#define DEFAULT_TORQUE_LIMIT 2.0f   // [Nm]
#define DEFAULT_VEL_LIMIT 1000.0f   // [rpm] (redefined from VEL_LIMIT_DEFAULT)
#define DEFAULT_VOLTAGE_LIMIT 58.0f // [V] (14S Battery Max)
#define DEFAULT_CURRENT_LIMIT 10.0f // [A]

/* 斜坡控制 */
#define DEFAULT_TORQUE_RAMP_RATE 10.0f // [Nm/s]
#define DEFAULT_VEL_RAMP_RATE 100.0f   // [rpm/s]
#define DEFAULT_TRAJ_VEL 20.0f         // [turn/s]
#define DEFAULT_TRAJ_ACCEL 100.0f      // [turn/s^2]
#define DEFAULT_TRAJ_DECEL 100.0f      // [turn/s^2]

/* PID 默认参数 (电流环) */
#define DEFAULT_CURRENT_P_GAIN 2.0f
#define DEFAULT_CURRENT_I_GAIN 100.0f
#define DEFAULT_CURRENT_BW 1000 // [Hz] or [rad/s]
#define DEFAULT_PID_MAX_OUT CURRENT_PID_MAX_OUT
#define DEFAULT_PID_MAX_IOUT 10.0f

/* PID 默认参数 (速度环) */
#define DEFAULT_VEL_P_GAIN 0.05f
#define DEFAULT_VEL_I_GAIN 1.0f
#define DEFAULT_VEL_D_GAIN 0.0f
#define DEFAULT_VEL_MAX_OUT VEL_PID_MAX_OUT
#define DEFAULT_VEL_MAX_IOUT 10.0f

/* PID 默认参数 (位置环) */
#define DEFAULT_POS_P_GAIN 20.0f
#define DEFAULT_POS_I_GAIN 0.0f
#define DEFAULT_POS_D_GAIN 0.0f
#define DEFAULT_POS_MAX_OUT POS_PID_MAX_OUT
#define DEFAULT_POS_MAX_IOUT 10.0f

/* MIT 默认参数 */
#define DEFAULT_MIT_KP 0.0f
#define DEFAULT_MIT_KD 0.0f

/* ==============================================================================
   6. 保护阈值配置 (Protection Thresholds)
   ==============================================================================
 */
#define DEFAULT_VBUS_VOLTAGE_V 52.0f // 默认母线电压 [V] (14S Nominal)
#define VBUS_MIN_VALID_V 5.0f        // 最小有效母线电压 [V]
/* 注意: 故障阈值和检测常量已移至 ALGO/motor/fault_def.h */

/* ==============================================================================
   6. 数学常量 (Math Constants)
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
