/**
 * @file motor_data.c
 * @brief Global motor data and state definitions
 */

#include "fsm.h"
#include "motor.h"
#include "config.h"
#include "foc/foc_algorithm.h"
#include "motor_hal_api.h"

#include "mt6816_encoder.h"

extern Motor_HAL_Handle_t g431_hal_handle;
extern MT6816_Handle_t encoder_data;

/* 定义全局DS402状态机实例 */
StateMachine g_ds402_state_machine;

/* 定义全局CAN配置 */
uint8_t g_can_id = DEFAULT_CAN_ID;
uint8_t g_can_baudrate = DEFAULT_CAN_BAUDRATE;
uint8_t g_protocol_type = DEFAULT_PROTOCOL_TYPE;
uint32_t g_can_timeout_ms = DEFAULT_CAN_TIMEOUT_MS;

/* 定义全局功能配置 */
uint8_t g_zero_sta = DEFAULT_ZERO_STA;
float g_add_offset = DEFAULT_ADD_OFFSET;
uint8_t g_damper_enable = DEFAULT_DAMPER_ENABLE;
uint8_t g_run_mode = DEFAULT_RUN_MODE;

/**
 * @brief 全局电机数据结构
 */
MOTOR_DATA motor_data = {
    .components =
        {
            .hal = &g431_hal_handle,
            .encoder = &encoder_data,
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
    .algo_config = {0},
    .algo_input = {0},
    .algo_output = {0},
};
