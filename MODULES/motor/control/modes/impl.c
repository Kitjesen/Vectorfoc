#include "impl.h"
#include "clarke.h"
#include "hal_pwm.h"
#include "motor/config.h"
#include "motor/control/control.h"
#include "motor/foc/foc_algorithm.h"
#include "motor/foc/park.h"
#include "motor/foc/svpwm.h"
#include "motor_hal_api.h"
#include "mt6816_encoder.h"
#include "trap_traj.h"
#include <math.h>

/**
 * @brief Control Mode Implementations
 */
extern void OpenControlMode(MOTOR_DATA *motor, float target_velocity);

// Debug Macros (Legacy)
#ifndef TORQUE_ADJUST
#define TORQUE_ADJUST 0
#endif

#ifndef TORQUE_AND_CURRENT
#define TORQUE_AND_CURRENT 0
#endif

void ControlImpl_SetThetaFromEncoder(MOTOR_DATA *motor) {
  motor->algo_input.theta_elec = motor->feedback.phase_angle;
}

void ControlImpl_SetPidLimits(MOTOR_DATA *motor) {
  switch (motor->state.Control_Mode) {
  case CONTROL_MODE_OPEN:
    // Open loop mode does not use inner/outer loop PID limits
    break;
  case CONTROL_MODE_TORQUE:
    SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.0f, 0.0f,
                0.0f);
    break;
  case CONTROL_MODE_VELOCITY:
    SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT,
                VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, 0.0f);
    break;
  case CONTROL_MODE_POSITION:
    SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT,
                VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
    break;
  case CONTROL_MODE_VELOCITY_RAMP:
  case CONTROL_MODE_POSITION_RAMP:
    SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT,
                VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
    break;
  case CONTROL_MODE_MIT:
    SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.0f, 0.0f,
                0.0f);
    break;
  default:
    break;
  }
}

/**
 * @brief 注入电压矢量 (开环控制实现)
 * @note 替代旧的 FOC_voltage 函数，使用新架构的数据结构。
 */
void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle) {
  // 1. 设置目标电压
  motor->algo_output.Vd = Vd;
  motor->algo_output.Vq = Vq;
  motor->algo_input.theta_elec = angle;

  // 2. 逆Park变换 (dq -> alpha-beta)
  Park_Inverse(Vd, Vq, angle, &motor->algo_output.Valpha,
               &motor->algo_output.Vbeta);

  // 3. SVPWM调制
  // 注意：这里需要确保 Vbus 有效，否则使用默认值
  float vbus = motor->algo_input.Vbus;
  if (vbus < VBUS_MIN_VALID_V) {
    vbus = DEFAULT_VBUS_VOLTAGE_V;
  }

  SVPWM_Modulate(motor->algo_output.Valpha, motor->algo_output.Vbeta, vbus,
                 &motor->algo_output.Ta, &motor->algo_output.Tb,
                 &motor->algo_output.Tc);

  // 4. 应用到硬件 (HAL)
  MHAL_PWM_SetDuty(motor->algo_output.Ta, motor->algo_output.Tb,
                   motor->algo_output.Tc);
}

void ControlImpl_Open(MOTOR_DATA *motor) { OpenControlMode(motor, 5); }

void ControlImpl_Torque(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  ControlImpl_SetThetaFromEncoder(motor);

#if TORQUE_AND_CURRENT
#if TORQUE_ADJUST
  motor->algo_input.Iq_ref = 0.0f;
  motor->algo_input.Id_ref = 0.5f;
#else
  motor->Controller.input_current =
      CLAMP(motor->Controller.input_current, -motor->Controller.current_limit,
            motor->Controller.current_limit);
  float max_step_size =
      fabsf(CURRENT_MEASURE_PERIOD * motor->Controller.torque_ramp_rate);
  float full_step =
      motor->Controller.input_current - motor->algo_input.Iq_ref;
  float step = CLAMP(full_step, -max_step_size, max_step_size);
  motor->algo_input.Iq_ref += step;
  motor->algo_input.Id_ref = 0.0f;
#endif
#else
  motor->Controller.input_torque =
      CLAMP(motor->Controller.input_torque, -motor->Controller.torque_limit,
            motor->Controller.torque_limit);
  motor->Controller.input_current =
      CLAMP(motor->Controller.input_torque / motor->Controller.torque_const,
            -motor->Controller.current_limit, +motor->Controller.current_limit);
  float max_step_size =
      fabsf(CURRENT_MEASURE_PERIOD * motor->Controller.torque_ramp_rate);
  float full_step =
      motor->Controller.input_current - motor->algo_input.Iq_ref;
  float step = CLAMP(full_step, -max_step_size, max_step_size);
  motor->algo_input.Iq_ref += step;
  motor->algo_input.Id_ref = 0.0f;
#endif
}

void ControlImpl_Velocity(MOTOR_DATA *motor) {
  ControlImpl_SetThetaFromEncoder(motor);
}

void ControlImpl_Position(MOTOR_DATA *motor) {
  ControlImpl_SetThetaFromEncoder(motor);
}

void ControlImpl_VelocityRamp(MOTOR_DATA *motor) {
  ControlImpl_SetThetaFromEncoder(motor);
  float max_step_size = fabsf(VEL_POS_PERIOD * motor->Controller.vel_ramp_rate);
  float full_step =
      motor->Controller.input_velocity - motor->Controller.vel_setpoint;
  float step = CLAMP(full_step, -max_step_size, max_step_size);
  motor->Controller.vel_setpoint += step;
  motor->Controller.torque_setpoint =
      (step / VEL_POS_PERIOD) * motor->Controller.inertia;
}

void ControlImpl_PositionRamp(MOTOR_DATA *motor, MotorControlCtx *ctx) {
  ControlImpl_SetThetaFromEncoder(motor);

  // 1) Trajectory Planning (When new target arrives)
  if (motor->Controller.input_updated) {
    TRAJ_plan(&ctx->traj, motor->Controller.input_position, // Target Position
              motor->feedback.position,                    // Current Position
              motor->feedback.velocity,                    // Current Velocity
              motor->Controller.traj_vel,                  // Max Velocity
              motor->Controller.traj_accel,                // Max Acceleration
              motor->Controller.traj_decel);               // Max Deceleration

    ctx->traj.t = 0.0f;
    ctx->traj.trajectory_done = false;
    motor->Controller.input_updated = false;
  }

  // Avoid updating uninitialized trajectory
  if (ctx->traj.trajectory_done) {
    return;
  }

  if (ctx->traj.t > ctx->traj.Tf_) {
    ctx->traj.trajectory_done = true;
    motor->Controller.pos_setpoint = motor->Controller.input_position;
    motor->Controller.vel_setpoint = 0.0f;
    motor->Controller.torque_setpoint = 0.0f;
  } else {
    TRAJ_eval(&ctx->traj, ctx->traj.t);
    motor->Controller.pos_setpoint = ctx->traj.Y;
    motor->Controller.vel_setpoint = ctx->traj.Yd;
    motor->Controller.torque_setpoint =
        ctx->traj.Ydd * motor->Controller.inertia;
    if (fabsf(motor->Controller.pos_setpoint - motor->feedback.position) <
        MIT_POSITION_ERROR_TOLERANCE) {
      ctx->traj.t += VEL_POS_PERIOD;
    }
  }
}

void ControlImpl_MIT(MOTOR_DATA *motor) {
  ControlImpl_SetThetaFromEncoder(motor);

  // ========== Parameter Validity Check ==========
  if (motor->Controller.mit_kp < 0.0f || motor->Controller.mit_kd < 0.0f) {
    motor->algo_input.Iq_ref = 0.0f;
    motor->algo_input.Id_ref = 0.0f;
    return;
  }

  // ========== Unit Conversion: turn -> rad ==========
  float pos_actual_rad = motor->feedback.position * M_2PI;
  float vel_actual_rad = motor->feedback.velocity * M_2PI;

  // ========== MIT Impedance Control: τ = Kp*Δθ + Kd*Δω ==========
  float pos_error = motor->Controller.mit_pos_des - pos_actual_rad;
  float vel_error = motor->Controller.mit_vel_des - vel_actual_rad;

  // ========== Stability Check: Protect if error is too large ==========
  if (fabsf(pos_error) > MIT_POSITION_STABILITY_THRESH ||
      fabsf(vel_error) > MIT_VELOCITY_STABILITY_THRESH) {
    motor->algo_input.Iq_ref *= MIT_MODE_DECAY_FACTOR;
    motor->algo_input.Id_ref = 0.0f;
    return;
  }

  // Calculate Impedance Torque
  float impedance_torque = motor->Controller.mit_kp * pos_error +
                           motor->Controller.mit_kd * vel_error;

  // ========== Torque Limiting ==========
  float desired_torque =
      CLAMP(impedance_torque, -motor->Controller.torque_limit,
            motor->Controller.torque_limit);

  // ========== Convert to Current and Limit ==========
  float desired_current = desired_torque / motor->Controller.torque_const;
  motor->Controller.input_current =
      CLAMP(desired_current, -motor->Controller.current_limit,
            motor->Controller.current_limit);

  motor->algo_input.Iq_ref = motor->Controller.input_current;
  motor->algo_input.Id_ref = 0.0f;
}

/**
 * @brief 开环控制模式 (速度积分控制电角度)
 * @param motor 电机数据指针
 * @param target_velocity 目标速度 [rad/s]
 *
 * @note 控制原理:
 *   1. 根据目标速度积分计算电角度: θ = θ + ω×Ts
 *   2. 施加固定幅值的电压矢量 (Vd=0, Vq=0.5V)
 *   3. 适用于无传感器启动或低速运行
 */
void OpenControlMode(MOTOR_DATA *motor, float target_velocity) {
  float Ts = CURRENT_MEASURE_PERIOD; // 控制周期 [s]

  // 电角度积分: θ(k+1) = θ(k) + ω×Ts
  // 使用新架构变量
  motor->algo_input.theta_elec = normalize_angle(
      motor->algo_input.theta_elec + target_velocity * Ts);

  // 施加固定电压矢量 (Vd=0, Vq=固定值)
  // 替换 FOC_voltage 为 Control_InjectVoltage
  Control_InjectVoltage(motor, 0.0f, OPEN_MODE_FIXED_VOLTAGE,
                        motor->algo_input.theta_elec);
}
