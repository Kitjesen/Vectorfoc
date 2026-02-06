#ifndef CONTROL_MODES_IMPL_H
#define CONTROL_MODES_IMPL_H

#include "context.h"
#include "motor.h"

// Mode Implementations
void ControlImpl_Open(MOTOR_DATA *motor);
void ControlImpl_Torque(MOTOR_DATA *motor, MotorControlCtx *ctx);
void ControlImpl_Velocity(MOTOR_DATA *motor);
void ControlImpl_Position(MOTOR_DATA *motor);
void ControlImpl_VelocityRamp(MOTOR_DATA *motor);
void ControlImpl_PositionRamp(MOTOR_DATA *motor, MotorControlCtx *ctx);
void ControlImpl_MIT(MOTOR_DATA *motor);

// Helpers
void ControlImpl_SetThetaFromEncoder(MOTOR_DATA *motor);
void ControlImpl_SetPidLimits(MOTOR_DATA *motor);

/**
 * @brief 注入电压矢量 (开环控制)
 * @note 用于校准和开环测试，绕过PID控制直接输出电压。
 * @param motor 电机数据指针
 * @param Vd D轴电压 [V]
 * @param Vq Q轴电压 [V]
 * @param angle 电角度 [rad]
 */
void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle);

#endif // CONTROL_MODES_IMPL_H
