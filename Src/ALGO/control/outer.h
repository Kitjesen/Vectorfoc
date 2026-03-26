#ifndef CONTROL_OUTER_H
#define CONTROL_OUTER_H
#include "context.h"
#include "motor.h"
/**
 * @brief runningouter loop
 */
bool Control_ShouldRunOuterLoops(const MOTOR_DATA *motor);
/**
 * @brief updateouter loop（speed/velocity/position）
 */
void Control_OuterLoopsUpdate(MOTOR_DATA *motor, MotorControlCtx *ctx);
#endif // CONTROL_OUTER_H
