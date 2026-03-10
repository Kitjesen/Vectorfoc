#ifndef TEST_MOCK_MOTOR_H
#define TEST_MOCK_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

extern uint8_t g_can_id;
extern uint8_t g_can_baudrate;
extern uint8_t g_protocol_type;
extern uint32_t g_can_timeout_ms;
extern uint8_t g_run_mode;

typedef enum {
  CONTROL_MODE_OPEN = 0,
  CONTROL_MODE_TORQUE = 1,
  CONTROL_MODE_VELOCITY = 2,
  CONTROL_MODE_POSITION = 3,
  CONTROL_MODE_VELOCITY_RAMP = 4,
  CONTROL_MODE_POSITION_RAMP = 5,
  CONTROL_MODE_MIT = 6,
} CONTROL_MODE;

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float out;
} PidTypeDef;

typedef struct {
  float omega_o;
  float omega_c;
  float b0;
  float max_output;
} LADRC_Config_t;

typedef struct {
  float output;
  bool initialized;
} LADRC_State_t;

typedef struct {
  float Ia;
  float Ib;
  float Ic;
  float Iq_ref;
  float Id_ref;
  float Vbus;
} FOC_AlgorithmInput_t;

typedef struct {
  float Iq;
  float Id;
} FOC_AlgorithmOutput_t;

typedef struct {
  CONTROL_MODE Control_Mode;
  uint8_t State_Mode;
  uint8_t Sub_State;
  uint8_t Cs_State;
  uint8_t Fault_State;
} MOTOR_STATE;

typedef struct {
  float input_position;
  float input_velocity;
  float input_torque;
  float input_current;
  float pos_setpoint;
  float vel_setpoint;
  float torque_setpoint;
  float vel_limit;
  float current_limit;
  int current_ctrl_bandwidth;
  float mit_kp;
  float mit_kd;
  float mit_pos_des;
  float mit_vel_des;
  bool input_updated;
} MOTOR_CONTROLLER;

typedef struct {
  float position;
  float velocity;
  float temperature;
} MOTOR_FEEDBACK;

typedef struct {
  float Rs;
  float Ls;
  float flux;
  int pole_pairs;
} MOTOR_PARAMETERS;

typedef struct MOTOR_DATA_s {
  MOTOR_STATE state;
  MOTOR_CONTROLLER Controller;
  MOTOR_FEEDBACK feedback;
  MOTOR_PARAMETERS parameters;
  PidTypeDef IqPID;
  PidTypeDef IdPID;
  PidTypeDef VelPID;
  PidTypeDef PosPID;
  FOC_AlgorithmInput_t algo_input;
  FOC_AlgorithmOutput_t algo_output;
  LADRC_Config_t ladrc_config;
  LADRC_State_t ladrc_state;
  float ladrc_enable;
  struct {
    float smo_alpha;
    float smo_beta;
    float ff_friction;
    float fw_max_current;
    float fw_start_velocity;
    float cogging_comp_enabled;
    float cogging_calib_request;
  } advanced;
  volatile bool params_updated;
} MOTOR_DATA;

extern MOTOR_DATA motor_data;

#endif
