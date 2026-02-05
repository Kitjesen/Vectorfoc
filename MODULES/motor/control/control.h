#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor.h"

// Forward declaration if motor.h inclusion is guarded
struct MOTOR_DATA_s;
// typedef struct MOTOR_DATA_s MOTOR_DATA; // Removed to avoid redefinition

/**
 * @brief  Initialize Motor Control Context
 * @param  motor Motor Control Data
 */
void Control_Init(MOTOR_DATA *motor);

/**
 * @brief Motor Control Main Dispatch Entry
 *
 * Coordinates the various parts of the control system:
 * 1. Rate Limiting - Process input commands
 * 2. Mode Dispatching - Delegate to modes/impl
 * 3. Outer Loop - Velocity/Position Control (loops/outer)
 * 4. Inner Loop - FOC Current Control (loops/inner)
 *
 * @param motor Motor Control Data
 */
void MotorControl_Run(MOTOR_DATA *motor);

/**
 * @brief 设置 PID 控制器限幅
 */
void SetPIDLimit(MOTOR_DATA *motor, float current_max_out,
                 float current_max_iout, float vel_max_out, float vel_max_iout,
                 float pos_limit);

/**
 * @brief 更新电流环控制参数 (增益与限幅)
 */
void CurrentLoop_UpdateGain(MOTOR_DATA *motor);

#endif // MOTOR_CONTROL_H
