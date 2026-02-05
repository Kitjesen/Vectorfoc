#ifndef CONTROL_INNER_H
#define CONTROL_INNER_H

#include "context.h"
#include "motor.h"

/**
 * @brief 电流内环控制：从 id/iq 到 vd/vq 的转换
 * @param motor 电机数据结构体指针
 * @param ctx 内部控制上下文
 */
void Control_InnerCurrentLoop(MOTOR_DATA *motor, MotorControlCtx *ctx);

#endif // CONTROL_INNER_H
