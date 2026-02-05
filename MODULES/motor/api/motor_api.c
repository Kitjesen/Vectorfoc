#include "motor_api.h"
#include "current_calib.h"
#include "motor/control/control.h" // For CurrentLoop_UpdateGain, Control_Init
#include "motor/control/features/cogging.h"
#include "motor/control/features/feedforward.h"
#include "motor/control/features/field_weakening.h"
#include "motor/hal/motor_hal_api.h" // For MHAL_PWM_Brake
#include "motor/observer/smo_observer.h"
#include "motor_pwm_driver.h"
#include "param_access.h"
#include "param_table.h"
#include "rsls_calib.h"
#include "safety_control.h"

// External reference to the main motor data structure
extern MOTOR_DATA motor_data;

// =============================================================================
// Lifecycle / Management API
// =============================================================================

// Motor_API_Init removed - SMO is lazy-initialized in
// Motor_API_Observer_Update()

void Init_Motor_No_Calib(MOTOR_DATA *motor) {
  // 1. 初始化参数系统 (仅执行一次)
  Param_SystemInitOnce();

  // 2.1 初始化控制上下文 (限速器, FOC状态)
  Control_Init(motor);

  // 3. 初始化PWM硬件驱动
  Motor_PWM_Driver_Init();

  // 4. 设置初始状态为运行模式
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.Cs_State = CS_STATE_IDLE;
  // motor->state.State_Mode = STATE_MODE_RUNNING; // Legacy

  // 请求 FSM 进入运行状态
  StateMachine_RequestState(&g_ds402_state_machine, STATE_OPERATION_ENABLED);

  // 5. 根据加载的参数更新电流环增益
  CurrentLoop_UpdateGain(motor);

  // 6. 标记参数已更新 (确保内环首次加载)
  motor->params_updated = true;
}

void Init_Motor_Calib(MOTOR_DATA *motor) {
  // motor->components.encoder->calib_valid = false;
  motor->state.Sub_State =
      RSLS_CALIBRATING; // 切换到 Rs/Ls/方向/极对数/编码器校准
  motor->state.Cs_State = CS_MOTOR_R_START; // 开始电阻测量阶段
}

void Motor_RequestCalibration(MOTOR_DATA *motor, uint8_t calibration_type) {
  // 1. 关闭PWM输出 (安全第一)
  MHAL_PWM_Brake();

  // 2. 复位所有PID积分项 (防止再次启动时突变)
  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);

  // 3. 重置FOC状态
  FOC_Algorithm_ResetState(&motor->algo_state);
  CalibContext_Reset(&motor->calib_ctx);

  // 4. 设置校准子状态
  // 4. Select calibration mode
  switch (calibration_type) {
  case 1:
    motor->state.Sub_State = RSLS_CALIBRATING;
    RSLSCalib_Start(motor, &motor->calib_ctx);
    break;
  case 2:
    motor->state.Sub_State = RSLS_CALIBRATING;
    RSLSCalib_Start(motor, &motor->calib_ctx);
    break;
  default:
    motor->state.Sub_State = CURRENT_CALIBRATING;
    CurrentCalib_Start(motor, &motor->calib_ctx);
    break;
  }

  // 5. 切换主状态进入检测模式 (通过 FSM 请求)
  // motor->state.State_Mode = STATE_MODE_DETECTING; // Legacy
  StateMachine_RequestState(&g_ds402_state_machine, STATE_CALIBRATING);
}

void Motor_ClearFaults(MOTOR_DATA *motor) {
  if (motor->state.State_Mode == STATE_MODE_GUARD) {
    // 1. 清除故障码
    motor->state.Fault_State = FAULT_STATE_NORMAL;

    // 2. 复位 PID (以防万一)
    PID_clear(&motor->IqPID);
    PID_clear(&motor->IdPID);
    PID_clear(&motor->VelPID);
    PID_clear(&motor->PosPID);

    // 3. 恢复为 IDLE 状态
    motor->state.State_Mode = STATE_MODE_IDLE;

    // 4. 重置 LED 指示 (Assuming RGB_DisplayColorById is available via some
    // include, or need to verify) RGB_DisplayColorById(3); // 暂时注释，如果
    // headers incomplete. Wait, led.h is in motor.c but not here. Let's include
    // it.
  }
}

// =============================================================================
// Tuning / Configuration Implementations
// =============================================================================

void Motor_API_ConfigSMO(float alpha, float beta) {
  motor_data.advanced.smo_alpha = alpha;
  motor_data.advanced.smo_beta = beta;
}

void Motor_API_ConfigFeedforward(float friction_coeff) {
  motor_data.advanced.ff_friction = friction_coeff;
}

void Motor_API_ConfigFieldWeakening(float max_current, float start_velocity) {
  motor_data.advanced.fw_max_current = max_current;
  motor_data.advanced.fw_start_velocity = start_velocity;
}

void Motor_API_ConfigCogging(bool enable) {
  motor_data.advanced.cogging_comp_enabled = enable ? 1.0f : 0.0f;
}

void Motor_API_StartCoggingCalib(MOTOR_DATA *motor) {
  CoggingComp_StartCalibration(motor);
}

void Motor_API_StopCoggingCalib(MOTOR_DATA *motor) {
  CoggingComp_StopCalibration(motor);
}

bool Motor_API_IsCoggingCalibrating(void) {
  return CoggingComp_IsCalibrating();
}

bool Motor_API_IsCoggingMapValid(void) { return CoggingComp_IsValid(); }

// =============================================================================
// Task Hook Implementations
// =============================================================================

void Motor_API_Observer_Update(MOTOR_DATA *motor) {
  static SMO_Observer_t smo_state;
  static bool init = false;
  if (!init) {
    SMO_Observer_Init(&smo_state);
    init = true;
  }

  smo_state.alpha = motor->advanced.smo_alpha;
  smo_state.beta = motor->advanced.smo_beta;

  SMO_Observer_Update(&smo_state, motor);
}

void Motor_API_Feedforward_Update(MOTOR_DATA *motor) {
  Feedforward_Params_t params;
  params.friction_coeff = motor->advanced.ff_friction;
  params.inertia = motor->Controller.inertia;

  Feedforward_Update(motor, &params);
}

void Motor_API_FieldWeakening_Update(MOTOR_DATA *motor) {
  FieldWeakening_Config_t cfg;
  cfg.max_weakening_current = motor->advanced.fw_max_current;
  cfg.start_velocity = motor->advanced.fw_start_velocity;

  FieldWeakening_Update(motor, &cfg);
}

void Motor_API_Cogging_Update(MOTOR_DATA *motor) {
  if (motor->advanced.cogging_calib_request > 0.5f &&
      !CoggingComp_IsCalibrating()) {
    CoggingComp_StartCalibration(motor);
    motor->advanced.cogging_calib_request = 0.0f;
  }

  CoggingComp_Update(motor);
}
