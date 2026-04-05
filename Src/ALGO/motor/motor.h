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
 * @file motor.h
 * @brief motor
 *
 * motorphase、state。
 */
#ifndef MOTOR_H
#define MOTOR_H
#include "calibration_context.h"
#include "common.h"
#include "foc/foc_algorithm.h"
#include "fsm.h"
#include "motor_hal_api.h"
#include "control/ladrc.h"
#include "pid.h"
#ifndef BOARD_XSTAR
#include "mt6816_encoder.h"
#endif
#include <math.h>
/**
 * @brief  CAN ID
 */
/* CANconfig */
extern uint8_t g_can_id;
extern uint8_t g_can_baudrate;
extern uint8_t g_protocol_type;
extern uint32_t g_can_timeout_ms;
/* config */
extern uint8_t g_zero_sta;
extern float g_add_offset;
extern uint8_t g_damper_enable;
extern uint8_t g_run_mode;
/**
 * @brief mode
 *
 * motormode，open loop、、speed/velocity、positionmode。
 */
typedef enum {
  CONTROL_MODE_OPEN = 0,          /**< open loopmode */
  CONTROL_MODE_TORQUE = 1,        /**< mode */
  CONTROL_MODE_VELOCITY = 2,      /**< speed/velocitymode */
  CONTROL_MODE_POSITION = 3,      /**< positionmode */
  CONTROL_MODE_VELOCITY_RAMP = 4, /**< speed/velocitymode */
  CONTROL_MODE_POSITION_RAMP = 5, /**< positionmode */
  CONTROL_MODE_MIT = 6,           /**< MITmode */
} CONTROL_MODE;
/**
 * @brief calibrationstate
 *
 * motorcalibrationstate。
 */
typedef enum {
  SUB_STATE_IDLE = 0,  /**< idlestate */
  CURRENT_CALIBRATING, /**< currentcalibrationstate */
  RSLS_CALIBRATING,    /**< calibrationstate */
  FLUX_CALIBRATING,    /**< fluxcalibrationstate */
} SUB_STATE;
/**
 * @brief calibrationstatestate
 */
typedef enum {
  CS_STATE_IDLE = 0,
  CS_MOTOR_R_START,     /**< calibration */
  CS_MOTOR_R_LOOP,      /**< calibration */
  CS_MOTOR_R_END,       /**< calibration */
  CS_MOTOR_L_START,     /**< calibration */
  CS_MOTOR_L_LOOP,      /**< calibration */
  CS_MOTOR_L_END,       /**< calibration */
  CS_DIR_PP_START,      /**< /pole pairscalibration */
  CS_DIR_PP_LOOP,       /**< /pole pairscalibration */
  CS_DIR_PP_END,        /**< /pole pairscalibration */
  CS_ENCODER_START,     /**< encodercalibration */
  CS_ENCODER_CW_LOOP,   /**< encoder */
  CS_ENCODER_CCW_LOOP,  /**< encoder */
  CS_ENCODER_END,       /**< encodercalibration */
  CS_FLUX_START,        /**< fluxcalibration */
  CS_FLUX_LOOP,         /**< fluxcalibration */
  CS_FLUX_END,          /**< fluxcalibration */
  CS_REPORT_OFFSET_LUT, /**< offset */
} CS_STATE;
/**
 * @brief motorrunningstatemode
 */
typedef enum {
  STATE_MODE_IDLE = 0,  /**< idlemode */
  STATE_MODE_DETECTING, /**< mode */
  STATE_MODE_RUNNING,   /**< runningmode */
  STATE_MODE_GUARD,     /**< protectionmode */
} STATE_MODE;
/**
 * @brief faultstate
 * @deprecated  Safety_GetActiveFaultBits() getfault
 * @note fault，
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
 * @brief motor：HAL encoder
 */
typedef struct {
  const Motor_HAL_Handle_t *hal; /**<  */
  void *encoder;                 /**< encoder (calibration) */
} MOTOR_COMPONENTS;
/**
 * @brief ：encoder MT6816_Handle_t
 * @warning encoder MT6816_Handle_t*
 */
#define ENC(m) ((MT6816_Handle_t *)((m)->components.encoder))
/**
 * @brief motorparam
 */
typedef struct {
  float Rs;       /**<  [Ohm] */
  float Ls;       /**<  [H] */
  float flux;     /**< flux [V·s] */
  int pole_pairs; /**< pole pairs */
} MOTOR_PARAMETERS;
/**
 * @brief motorparam
 */
typedef struct {
  float inertia;              /**< [A/(turn/s^2)] */
  float torque_ramp_rate;     /**<  [Nm/s] */
  float vel_ramp_rate;        /**< speed/velocity [(turn/s)/s] */
  float traj_vel;             /**< speed/velocity [turn/s] */
  float traj_accel;           /**< speed/velocity [(turn/s)/s] */
  float traj_decel;           /**< speed/velocity [(turn/s)/s] */
  float vel_limit;            /**< speed/velocitylimit [turn/s] */
  float torque_const;         /**<  [Nm/A] */
  float torque_limit;         /**< limit [Nm] */
  float current_limit;        /**< currentlimit [A] */
  float voltage_limit;        /**< voltagelimit [V] */
  float current_ctrl_p_gain;  /**< () current P gain */
  float current_ctrl_i_gain;  /**< () current I gain */
  int current_ctrl_bandwidth; /**< current [rad/s] (100~2000) */
  float input_position; /**< inputposition */
  float input_velocity; /**< inputspeed/velocity */
  float input_torque;   /**< input */
  float input_current;  /**< inputcurrent */
  float pos_setpoint;    /**< position */
  float vel_setpoint;    /**< speed/velocity */
  float torque_setpoint; /**<  */
  // MITparam
  float mit_kp;      /**< MITposition [Nm/rad] */
  float mit_kd;      /**< MIT [Nm·s/rad] */
  float mit_pos_des; /**< MITposition [rad] */
  float mit_vel_des; /**< MITspeed/velocity [rad/s] */
  volatile bool input_updated; /**< inputparamupdate */
} MOTOR_CONTROLLER;
/**
 * @brief motorstate
 */
typedef struct {
  STATE_MODE State_Mode;     /**< motorstate */
  CONTROL_MODE Control_Mode; /**< mode */
  SUB_STATE Sub_State;       /**< calibrationstate */
  CS_STATE Cs_State;         /**< calibrationstate */
  FAULT_STATE Fault_State;   /**< @deprecated faultstate（，） */
} MOTOR_STATE;
/**
 * @brief motorfeedback
 */
typedef struct {
  float position;          /**< position [rad] */
  float velocity;          /**< speed/velocity [rad/s] */
  float phase_angle;       /**< angle [rad] */
  float temperature;       /**< temperature [degC] */
  float observer_angle;    /**< observerangle [rad] */
  float observer_velocity; /**< observerspeed/velocity [rad/s] */
} MOTOR_FEEDBACK;
/**
 * @brief motor
 * 、state、param、、feedbackPID。
 */
typedef struct MOTOR_DATA_s {
  MOTOR_COMPONENTS components; /**<  */
  MOTOR_STATE state;           /**< runningstate */
  MOTOR_PARAMETERS parameters; /**< motorparam */
  MOTOR_CONTROLLER Controller; /**< param */
  MOTOR_FEEDBACK feedback;     /**< feedback */
  PidTypeDef IqPID;  /**< current IQ axis PID */
  PidTypeDef IdPID;  /**< current ID axis PID */
  PidTypeDef VelPID; /**< speed/velocity PID */
  PidTypeDef PosPID; /**< position PID */
  /* === FOC Core Data (New Architecture) === */
  FOC_AlgorithmState_t algo_state;   /**< FOCstate (integral, filter) */
  FOC_AlgorithmConfig_t algo_config; /**< FOCconfig (gain, limit) */
  FOC_AlgorithmInput_t algo_input;   /**< FOCinput (, reference) */
  FOC_AlgorithmOutput_t algo_output; /**< FOCoutput (PWM, ) */
  /* === LADRC speed/velocity === */
  LADRC_Config_t ladrc_config;   /**< LADRC configparam */
  LADRC_State_t ladrc_state;     /**< LADRC state */
  float ladrc_enable;            /**< LADRC enable (0.0=PID, 1.0=LADRC) */
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
  CalibrationContext calib_ctx; /**< calibration */
  volatile bool params_updated; /**< param (inner loopparam) */
} MOTOR_DATA;
extern MOTOR_DATA motor_data;
/**
 * @brief calibrationmotorinit
 * @param motor motor
 */
void Init_Motor_No_Calib(MOTOR_DATA *motor);
/**
 * @brief calibrationmotorinit
 * @param motor motor
 */
void Init_Motor_Calib(MOTOR_DATA *motor);
/**
 * @brief FOCstate
 * @param motor motor
 */
void MotorStateTask(MOTOR_DATA *motor);
/**
 * @brief FOCprotection
 * @param motor motor
 */
void MotorGuardTask(MOTOR_DATA *motor);
/**
 * @brief motorcalibration
 * @param motor motor
 * @param calibration_type calibration
 */
void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t calibration_type);
/**
 * @brief motorfaultstate
 * @param motor motor
 */
void Motor_ClearFaults(MOTOR_DATA *motor);
/**
 * @brief DS402state
 *
 * CANopen DS402state。
 *  app_init.c init， task_guard.c protection。
 */
extern StateMachine g_ds402_state_machine;
#endif // MOTOR_H
