/**
 * @file motor.h
 * @brief 电机核心控制模块头文件
 *
 * 定义了电机控制相关的数据结构、状态枚举和核心函数接口。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "calibration_context.h"
#include "common.h"
#include "motor/foc/foc_algorithm.h"
#include "motor/fsm/fsm.h"
#include "motor_hal_api.h"

#include "pid.h"
#include <math.h>

/**
 * @brief 全局 CAN ID
 */
/* 全局CAN配置 */
extern uint8_t g_can_id;
extern uint8_t g_can_baudrate;
extern uint8_t g_protocol_type;
extern uint32_t g_can_timeout_ms;

/* 全局功能配置 */
extern uint8_t g_zero_sta;
extern float g_add_offset;
extern uint8_t g_damper_enable;
extern uint8_t g_run_mode;

/**
 * @brief 控制模式枚举
 *
 * 定义电机的各种控制模式，包括开环、力矩、速度、位置及其组合模式。
 */
typedef enum {
  CONTROL_MODE_OPEN = 0,          /**< 开环模式 */
  CONTROL_MODE_TORQUE = 1,        /**< 力矩控制模式 */
  CONTROL_MODE_VELOCITY = 2,      /**< 速度控制模式 */
  CONTROL_MODE_POSITION = 3,      /**< 位置控制模式 */
  CONTROL_MODE_VELOCITY_RAMP = 4, /**< 速度斜坡控制模式 */
  CONTROL_MODE_POSITION_RAMP = 5, /**< 位置斜坡控制模式 */
  CONTROL_MODE_MIT = 6,           /**< MIT阻抗控制模式 */
} CONTROL_MODE;

/**
 * @brief 校准子状态枚举
 *
 * 定义电机校准过程中的各个子状态。
 */
typedef enum {
  SUB_STATE_IDLE = 0,  /**< 空闲状态 */
  CURRENT_CALIBRATING, /**< 电流校准状态 */
  RSLS_CALIBRATING,    /**< 电阻电感校准状态 */
  FLUX_CALIBRATING,    /**< 磁链校准状态 */
} SUB_STATE;

/**
 * @brief 校准状态机详细状态
 */
typedef enum {
  CS_STATE_IDLE = 0,
  CS_MOTOR_R_START,     /**< 电阻校准开始 */
  CS_MOTOR_R_LOOP,      /**< 电阻校准循环 */
  CS_MOTOR_R_END,       /**< 电阻校准结束 */
  CS_MOTOR_L_START,     /**< 电感校准开始 */
  CS_MOTOR_L_LOOP,      /**< 电感校准循环 */
  CS_MOTOR_L_END,       /**< 电感校准结束 */
  CS_DIR_PP_START,      /**< 方向/极对数校准开始 */
  CS_DIR_PP_LOOP,       /**< 方向/极对数校准循环 */
  CS_DIR_PP_END,        /**< 方向/极对数校准结束 */
  CS_ENCODER_START,     /**< 编码器校准开始 */
  CS_ENCODER_CW_LOOP,   /**< 编码器顺时针旋转循环 */
  CS_ENCODER_CCW_LOOP,  /**< 编码器逆时针旋转循环 */
  CS_ENCODER_END,       /**< 编码器校准结束 */
  CS_FLUX_START,        /**< 磁链校准开始 */
  CS_FLUX_LOOP,         /**< 磁链校准循环 */
  CS_FLUX_END,          /**< 磁链校准结束 */
  CS_REPORT_OFFSET_LUT, /**< 上报偏移量查找表 */
} CS_STATE;

/**
 * @brief 电机运行状态模式
 */
typedef enum {
  STATE_MODE_IDLE = 0,  /**< 空闲模式 */
  STATE_MODE_DETECTING, /**< 检测模式 */
  STATE_MODE_RUNNING,   /**< 运行模式 */
  STATE_MODE_GUARD,     /**< 保护模式 */
} STATE_MODE;

/**
 * @brief 故障状态枚举
 * @deprecated 请使用 Safety_GetActiveFaultBits() 获取完整故障位掩码
 * @note 此枚举只能表示单一故障，已废弃但保留用于兼容旧代码
 */
typedef enum {
  FAULT_STATE_NORMAL = 0,
  FAULT_STATE_OVER_CURRENT,
  FAULT_STATE_OVER_VOLTAGE,
  FAULT_STATE_UNDER_VOLTAGE,
  FAULT_STATE_OVER_TEMPERATURE,
  FAULT_STATE_SPEEDING,
  FAULT_STATE_ENCODER_LOSS,
} FAULT_STATE;

/**
 * @brief 电机组件结构体：HAL 与编码器句柄
 */
typedef struct {
  const Motor_HAL_Handle_t *hal; /**< 硬件抽象层句柄 */
  void *encoder;                 /**< 具体编码器句柄 (用于校准等特有操作) */
} MOTOR_COMPONENTS;

/**
 * @brief 宏定义：将组件中的编码器转换为 MT6816_Handle_t 类型
 * @warning 假设编码器类型为 MT6816_Handle_t*
 */
#define ENC(m) ((MT6816_Handle_t *)((m)->components.encoder))

/**
 * @brief 电机物理参数结构体
 */
typedef struct {
  float Rs;       /**< 电阻 [Ohm] */
  float Ls;       /**< 电感 [H] */
  float flux;     /**< 磁链 [V·s] */
  int pole_pairs; /**< 极对数 */
} MOTOR_PARAMETERS;

/**
 * @brief 电机控制器参数结构体
 */
typedef struct {
  float inertia;              /**<转动惯量 [A/(turn/s^2)] */
  float torque_ramp_rate;     /**< 力矩斜率 [Nm/s] */
  float vel_ramp_rate;        /**< 速度斜率 [(turn/s)/s] */
  float traj_vel;             /**< 轨迹速度 [turn/s] */
  float traj_accel;           /**< 轨迹加速度 [(turn/s)/s] */
  float traj_decel;           /**< 轨迹减速度 [(turn/s)/s] */
  float vel_limit;            /**< 速度限制 [turn/s] */
  float torque_const;         /**< 力矩常数 [Nm/A] */
  float torque_limit;         /**< 力矩限制 [Nm] */
  float current_limit;        /**< 电流限制 [A] */
  float voltage_limit;        /**< 电压限制 [V] */
  float current_ctrl_p_gain;  /**< (自动) 电流环 P 增益 */
  float current_ctrl_i_gain;  /**< (自动) 电流环 I 增益 */
  int current_ctrl_bandwidth; /**< 电流环带宽 [rad/s] (100~2000) */

  float input_position; /**< 输入位置 */
  float input_velocity; /**< 输入速度 */
  float input_torque;   /**< 输入力矩 */
  float input_current;  /**< 输入电流 */

  float pos_setpoint;    /**< 位置设定值 */
  float vel_setpoint;    /**< 速度设定值 */
  float torque_setpoint; /**< 力矩设定值 */

  // MIT阻抗控制参数
  float mit_kp;      /**< MIT位置刚度 [Nm/rad] */
  float mit_kd;      /**< MIT阻尼系数 [Nm·s/rad] */
  float mit_pos_des; /**< MIT期望位置 [rad] */
  float mit_vel_des; /**< MIT期望速度 [rad/s] */

  volatile bool input_updated; /**< 输入参数更新标志 */
} MOTOR_CONTROLLER;

/**
 * @brief 电机状态结构体
 */
typedef struct {
  STATE_MODE State_Mode;     /**< 电机主状态 */
  CONTROL_MODE Control_Mode; /**< 控制模式 */
  SUB_STATE Sub_State;       /**< 校准子状态 */
  CS_STATE Cs_State;         /**< 详细校准状态 */
  FAULT_STATE Fault_State;   /**< @deprecated 故障状态（已废弃，仅用于兼容） */
} MOTOR_STATE;

/**
 * @brief 电机反馈数据结构体
 */
typedef struct {
  float position;          /**< 机械位置 [rad] */
  float velocity;          /**< 机械速度 [rad/s] */
  float phase_angle;       /**< 电角度 [rad] */
  float temperature;       /**< 温度 [degC] */
  float observer_angle;    /**< 观测器估算角度 [rad] */
  float observer_velocity; /**< 观测器估算速度 [rad/s] */
} MOTOR_FEEDBACK;

/**
 * @brief 电机数据主结构体
 * 包含组件、状态、参数、控制器、反馈及PID控制器。
 */
typedef struct MOTOR_DATA_s {
  MOTOR_COMPONENTS components; /**< 硬件组件 */
  MOTOR_STATE state;           /**< 运行状态 */
  MOTOR_PARAMETERS parameters; /**< 电机参数 */
  MOTOR_CONTROLLER Controller; /**< 控制器参数 */
  MOTOR_FEEDBACK feedback;     /**< 传感器反馈 */

  PidTypeDef IqPID;  /**< 电流环 IQ 轴 PID */
  PidTypeDef IdPID;  /**< 电流环 ID 轴 PID */
  PidTypeDef VelPID; /**< 速度环 PID */
  PidTypeDef PosPID; /**< 位置环 PID */

  /* === FOC Core Data (New Architecture) === */
  FOC_AlgorithmState_t algo_state;   /**< FOC算法状态 (积分项, 滤波器) */
  FOC_AlgorithmConfig_t algo_config; /**< FOC算法配置 (增益, 限制) */
  FOC_AlgorithmInput_t algo_input;   /**< FOC算法输入 (传感器, 参考值) */
  FOC_AlgorithmOutput_t algo_output; /**< FOC算法输出 (PWM, 中间变量) */

  /* Advanced Control Configs - Persisted parameters map here */
  struct {
    float smo_alpha;
    float smo_beta;
    float ff_friction;
    float fw_max_current;
    float fw_start_velocity;
    float cogging_comp_enabled;  // Use float for param system compatibility or
                                 // cast
    float cogging_calib_request; // 1.0f triggers anticogging calibration
  } advanced;

  CalibrationContext calib_ctx; /**< 校准过程上下文 */

  volatile bool params_updated; /**< 参数变更标志 (用于内环参数同步) */
} MOTOR_DATA;

extern MOTOR_DATA motor_data;

/**
 * @brief 无需校准的电机初始化
 * @param motor 电机数据结构体指针
 */
void Init_Motor_No_Calib(MOTOR_DATA *motor);

/**
 * @brief 包含校准的电机初始化
 * @param motor 电机数据结构体指针
 */
void Init_Motor_Calib(MOTOR_DATA *motor);

/**
 * @brief FOC主状态机任务
 * @param motor 电机数据结构体指针
 */
void MotorStateTask(MOTOR_DATA *motor);

/**
 * @brief FOC保护任务
 * @param motor 电机数据结构体指针
 */
void MotorGuardTask(MOTOR_DATA *motor);

/**
 * @brief 请求电机校准
 * @param motor 电机数据结构体指针
 * @param calibration_type 校准类型
 */
void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t calibration_type);

/**
 * @brief 清除电机故障状态
 * @param motor 电机数据结构体指针
 */
void Motor_ClearFaults(MOTOR_DATA *motor);

/**
 * @brief 全局DS402状态机
 *
 * 用于CANopen DS402协议的标准状态管理。
 * 在 robot.c 中初始化，在 motor_task.c 和保护系统中使用。
 */
extern StateMachine g_ds402_state_machine;

#endif // MOTOR_H
