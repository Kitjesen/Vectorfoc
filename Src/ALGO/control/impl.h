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
 * @brief voltage (open loop)
 * @note calibrationopen loop，PIDoutputvoltage。
 * @param motor motor
 * @param Vd Daxisvoltage [V]
 * @param Vq Qaxisvoltage [V]
 * @param angle angle [rad]
 */
void Control_InjectVoltage(MOTOR_DATA *motor, float Vd, float Vq, float angle);
#endif // CONTROL_MODES_IMPL_H
