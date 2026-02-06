#include "control.h"
#include "context.h"
#include "inner.h"
#include "outer.h"
#include "impl.h"
#include "config.h"
#include "foc/foc_algorithm.h"
#include "config.h"
#include "trajectory/rate_limiter.h"

// Static Context
static MotorControlCtx s_ctx;

// Rate Limiters
static RateLimiterTypeDef s_vel_limiter;    // Velocity Rate Limiter
static RateLimiterTypeDef s_torque_limiter; // Torque Rate Limiter
static bool s_limiters_initialized = false;

void Control_Init(MOTOR_DATA *motor) {
  if (s_limiters_initialized) {
    return;
  }

  // 速度限速器: 最大加速度 = vel_limit * VELOCITY_ACCEL_MULTIPLIER
  RateLimiter_Init(&s_vel_limiter,
                   motor->Controller.vel_limit * VELOCITY_ACCEL_MULTIPLIER);
  // 力矩限速器: 使用配置的斜率
  RateLimiter_Init(&s_torque_limiter, motor->Controller.torque_ramp_rate);

  // 初始化FOC算法状态 (现在存储在 MOTOR_DATA 中)
  FOC_Algorithm_InitState(&motor->algo_state);

  s_limiters_initialized = true;
}

void MotorControl_Run(MOTOR_DATA *motor) {

  // 应用限速器
  motor->Controller.input_velocity = RateLimiter_Apply(
      &s_vel_limiter, motor->Controller.input_velocity, CURRENT_MEASURE_PERIOD);

  motor->Controller.input_torque =
      RateLimiter_Apply(&s_torque_limiter, motor->Controller.input_torque,
                        CURRENT_MEASURE_PERIOD);

  // 模式特定的逻辑: 限制, 轨迹, 设定值生成
  ControlImpl_SetPidLimits(motor);

  switch (motor->state.Control_Mode) {
  // 开环
  case CONTROL_MODE_OPEN:
    ControlImpl_Open(motor);
    break;
  // 力矩环
  case CONTROL_MODE_TORQUE:
    ControlImpl_Torque(motor, &s_ctx);
    break;
  // 速度环
  case CONTROL_MODE_VELOCITY:
    ControlImpl_Velocity(motor);
    break;
  // 位置环
  case CONTROL_MODE_POSITION:
    ControlImpl_Position(motor);
    break;
  // 速度斜坡
  case CONTROL_MODE_VELOCITY_RAMP:
    ControlImpl_VelocityRamp(motor);
    break;
  // 位置斜坡
  case CONTROL_MODE_POSITION_RAMP:
    ControlImpl_PositionRamp(motor, &s_ctx);
    break;
  // MIT : kp * (error) + kd * (error_rate)
  case CONTROL_MODE_MIT:
    ControlImpl_MIT(motor);
    break;
  default:
    break;
  }

  // 循环调度
  // 开环模式跳过FOC内环
  if (motor->state.Control_Mode <= CONTROL_MODE_OPEN) {
    return;
  }

  // 运行外环(速度/位置)
  if (Control_ShouldRunOuterLoops(motor)) {
    Control_OuterLoopsUpdate(motor, &s_ctx);
  }

  // 运行内环(电流FOC)
  Control_InnerCurrentLoop(motor, &s_ctx);
}

/**
 * @brief 设置 PID 控制器限幅
 *
 * @param motor 指向电机数据结构体的指针
 * @param current_max_out 电流环输出限幅 (通常为母线电压)
 * @param current_max_iout 电流环积分限幅
 * @param vel_max_out 速度环输出限幅 (通常为最大电流)
 * @param vel_max_iout 速度环积分限幅
 * @param pos_limit 位置环输出限幅 (通常为最大速度)
 */
void SetPIDLimit(MOTOR_DATA *motor, float current_max_out,
                 float current_max_iout, float vel_max_out, float vel_max_iout,
                 float pos_limit) {
  // D轴和Q轴共享相同的电压限幅
  motor->IdPID.max_out = current_max_out;
  motor->IdPID.max_iout = current_max_iout;

  motor->IqPID.max_out = current_max_out;
  motor->IqPID.max_iout = current_max_iout;

  motor->VelPID.max_out = vel_max_out;
  motor->VelPID.max_iout = vel_max_iout;

  motor->PosPID.max_out = pos_limit;
  motor->PosPID.max_iout = pos_limit;
}

/**
 * @brief 更新电流环控制参数 (增益与限幅)
 */
#define CURRENT_AUTO_CALIBRATION 1 // 手动电流调节还是自动调节
void CurrentLoop_UpdateGain(MOTOR_DATA *motor) {
  // 1. 计算增益
#if CURRENT_AUTO_CALIBRATION
  // 自动计算 (BW ≈ vel_limit * pole_pairs * 2pi)
  // 注意: 这里的带宽计算可能偏大，需根据实际情况调整
  float bandwidth =
      motor->Controller.vel_limit * motor->parameters.pole_pairs * M_2PI;
  motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * bandwidth;
  motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * bandwidth;
#else
  // 手动带宽
  motor->Controller.current_ctrl_p_gain =
      motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth * 1.0f;
  motor->Controller.current_ctrl_i_gain =
      motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth;
#endif

  // 2. 应用增益到控制器
  motor->IdPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IdPID.Ki = motor->Controller.current_ctrl_i_gain;
  motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
  motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;

  // 3. 应用限幅 (防止限幅为0导致无输出)
  // D/Q轴电压限幅 = voltage_limit
  float v_limit = motor->Controller.voltage_limit;
  motor->IdPID.max_out = v_limit;
  motor->IdPID.max_iout = v_limit;
  motor->IqPID.max_out = v_limit;
  motor->IqPID.max_iout = v_limit;

  // 速度/位置环限幅更新建议在模式切换时处理
  motor->VelPID.max_out = motor->Controller.current_limit;
  motor->VelPID.max_iout = motor->Controller.current_limit;
  motor->PosPID.max_out = motor->Controller.vel_limit;
  motor->PosPID.max_iout = motor->Controller.vel_limit;
}
