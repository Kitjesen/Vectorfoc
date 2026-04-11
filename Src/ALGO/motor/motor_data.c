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
 * @file motor_data.c
 * @brief Global motor data and state definitions
 */
#include "fsm.h"
#include "motor.h"
#include "config.h"   // 间接包含 board_config.h → HW_POSITION_SENSOR_MODE / HW_MOTOR_HAL_HANDLE
#include "foc/foc_algorithm.h"
#include "motor_hal_api.h"
/* 根据位置传感器类型引入对应驱动头文件及实例声明 */
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
#include "hall_encoder.h"
extern Motor_HAL_Handle_t xstar_hal_handle;
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
#include "abz_encoder.h"
extern Motor_HAL_Handle_t xstar_hal_handle;
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#include "tmr3109_encoder.h"
extern Motor_HAL_Handle_t g431_hal_handle;
extern TMR3109_Handle_t tmr3109_encoder_data;
#else  /* HW_POSITION_SENSOR_MT6816（默认）*/
#include "mt6816_encoder.h"
extern Motor_HAL_Handle_t g431_hal_handle;
extern MT6816_Handle_t encoder_data;
#endif
extern Motor_HAL_Handle_t HW_MOTOR_HAL_HANDLE;
/* DS402state */
StateMachine g_ds402_state_machine;
/* CANconfig */
uint8_t g_can_id = DEFAULT_CAN_ID;
uint8_t g_can_baudrate = DEFAULT_CAN_BAUDRATE;
uint8_t g_protocol_type = DEFAULT_PROTOCOL_TYPE;
uint32_t g_can_timeout_ms = DEFAULT_CAN_TIMEOUT_MS;
/* config */
uint8_t g_zero_sta = DEFAULT_ZERO_STA;
float g_add_offset = DEFAULT_ADD_OFFSET;
uint8_t g_damper_enable = DEFAULT_DAMPER_ENABLE;
uint8_t g_run_mode = DEFAULT_RUN_MODE;
/**
 * @brief motor
 */
MOTOR_DATA motor_data = {
    .components =
        {
            .hal = &HW_MOTOR_HAL_HANDLE,
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
            .encoder = &hall_data,
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
            .encoder = &abz_data,
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
            .encoder = &tmr3109_encoder_data,
#else  /* HW_POSITION_SENSOR_MT6816（默认）*/
            .encoder = &encoder_data,
#endif
        },
    .state =
        {
            .State_Mode = STATE_MODE_IDLE,
            .Control_Mode = CONTROL_MODE_VELOCITY_RAMP,
            .Sub_State = SUB_STATE_IDLE,
            .Cs_State = CS_STATE_IDLE,
            .Fault_State = FAULT_STATE_NORMAL,
        },
    .parameters =
        {
            .Rs = DEFAULT_RS,
            .Ls = DEFAULT_LS,
            .flux = DEFAULT_FLUX,
            .pole_pairs = DEFAULT_POLE_PAIRS,
        },
    .Controller =
        {
            .inertia = DEFAULT_INERTIA,
            .torque_ramp_rate = DEFAULT_TORQUE_RAMP_RATE,
            .vel_ramp_rate = DEFAULT_VEL_RAMP_RATE,
            .traj_vel = DEFAULT_TRAJ_VEL,
            .traj_accel = DEFAULT_TRAJ_ACCEL,
            .traj_decel = DEFAULT_TRAJ_DECEL,
            .torque_const = DEFAULT_TORQUE_CONST,
            .torque_limit = DEFAULT_TORQUE_LIMIT,
            .vel_limit = DEFAULT_VEL_LIMIT,
            .voltage_limit = DEFAULT_VOLTAGE_LIMIT,
            .current_limit = DEFAULT_CURRENT_LIMIT,
            .current_ctrl_p_gain = DEFAULT_CURRENT_P_GAIN,
            .current_ctrl_i_gain = DEFAULT_CURRENT_I_GAIN,
            .current_ctrl_bandwidth = DEFAULT_CURRENT_BW,
            .input_current = 0.0f,
            .input_torque = 0.0f,
            .input_velocity = 0.0f,
            .input_position = 0.0f,
            .input_updated = true,
            .mit_kp = DEFAULT_MIT_KP,
            .mit_kd = DEFAULT_MIT_KD,
            .mit_pos_des = 0.0f,
            .mit_vel_des = 0.0f,
        },
    .IqPID =
        {
            .mode = PID_POSITION, // Note: Enum name might be slightly
                                  // misleading if generic, but keeping as is
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .max_out = DEFAULT_PID_MAX_OUT,
            .max_iout = DEFAULT_PID_MAX_IOUT,
        },
    .IdPID =
        {
            .mode = PID_POSITION,
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .max_out = DEFAULT_PID_MAX_OUT,
            .max_iout = DEFAULT_PID_MAX_IOUT,
        },
    .VelPID =
        {
            .mode = PID_POSITION,
            .Kp = DEFAULT_VEL_P_GAIN,
            .Ki = DEFAULT_VEL_I_GAIN,
            .Kd = DEFAULT_VEL_D_GAIN,
            .max_out = DEFAULT_VEL_MAX_OUT,
            .max_iout = DEFAULT_VEL_MAX_IOUT,
        },
    .PosPID =
        {
            .mode = PID_POSITION,
            .Kp = DEFAULT_POS_P_GAIN,
            .Ki = DEFAULT_POS_I_GAIN,
            .Kd = DEFAULT_POS_D_GAIN,
            .max_out = DEFAULT_POS_MAX_OUT,
            .max_iout = DEFAULT_POS_MAX_IOUT,
        },
    // New Architecture Initialization
    .algo_state = {0},
    .algo_config =
        {
            .Rs = DEFAULT_RS,
            .Ls = DEFAULT_LS,
            .flux = DEFAULT_FLUX,
            .pole_pairs = DEFAULT_POLE_PAIRS,
            .Kp_current_d = DEFAULT_CURRENT_P_GAIN,
            .Ki_current_d = DEFAULT_CURRENT_I_GAIN,
            .Kp_current_q = DEFAULT_CURRENT_P_GAIN,
            .Ki_current_q = DEFAULT_CURRENT_I_GAIN,
            .Ts_current = CURRENT_MEASURE_PERIOD,
            .voltage_limit = DEFAULT_VOLTAGE_LIMIT,
            .current_limit = DEFAULT_CURRENT_LIMIT,
            .enable_decoupling = true,
            .decoupling_gain = 1.0f,
            .Kb_current = 1.0f,                  /* Back-calculation gain */
            .current_filter_fc = 0.0f,           /* Disabled: avoid phase lag, rely on hardware RC filter */
            .deadtime_i_thresh = 0.2f,           /* 电流过零阈值 [A]，ADC 噪声的 2~3 倍 */
            .deadtime_Vdiode = 0.7f,             /* 体二极管压降 [V] */
        },
    .algo_input = {0},
    .algo_output = {0},
    // [FIX] 初始化 LADRC 配置
    .ladrc_config =
        {
            .omega_o = DEFAULT_LADRC_OMEGA_O,
            .omega_c = DEFAULT_LADRC_OMEGA_C,
            .b0 = DEFAULT_LADRC_B0,
            .max_output = DEFAULT_LADRC_MAX_OUT,
        },
    .ladrc_state = {0},
    .ladrc_enable = (float)DEFAULT_LADRC_ENABLE,
    // [FIX] 初始化高级控制参数
    .advanced =
        {
            .smo_alpha = 0.0f,
            .smo_beta = 0.0f,
            .ff_friction = 0.0f,
            .fw_max_current = 0.0f,
            .fw_start_velocity = 0.0f,
            .cogging_comp_enabled = 0.0f,
            .cogging_calib_request = 0.0f,
        },
    .params_updated = true,
};
