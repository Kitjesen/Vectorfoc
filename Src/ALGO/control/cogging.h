#ifndef COGGING_H
#define COGGING_H

#include "motor.h"

void CoggingComp_Update(MOTOR_DATA *motor);
float CoggingComp_GetCurrent(const MOTOR_DATA *motor);
void CoggingComp_StartCalibration(MOTOR_DATA *motor);
void CoggingComp_StopCalibration(MOTOR_DATA *motor);
bool CoggingComp_IsCalibrating(void);
bool CoggingComp_IsValid(void);

#endif // COGGING_H
