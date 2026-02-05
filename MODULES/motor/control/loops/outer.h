#ifndef CONTROL_OUTER_H
#define CONTROL_OUTER_H

#include "context.h"
#include "motor.h"

/**
 * @brief 判断是否应该运行外环
 */
bool Control_ShouldRunOuterLoops(const MOTOR_DATA *motor);

/**
 * @brief 更新外环（速度/位置）
 */
void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx);

#endif // CONTROL_OUTER_H
