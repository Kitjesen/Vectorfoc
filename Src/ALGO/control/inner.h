#ifndef CONTROL_INNER_H
#define CONTROL_INNER_H
#include "context.h"
#include "motor.h"
/**
 * @brief currentinner loop： id/iq  vd/vq
 * @param motor motor
 * @param ctx
 */
void Control_InnerCurrentLoop(MOTOR_DATA *motor, MotorControlCtx *ctx);
#endif // CONTROL_INNER_H
