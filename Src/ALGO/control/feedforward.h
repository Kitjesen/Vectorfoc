#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include "motor.h"

// Feedforward Parameters
typedef struct {
  float inertia;
  float friction_coeff;
  // Add other FF parameters if needed
} Feedforward_Params_t;

void Feedforward_Init(Feedforward_Params_t *params);
void Feedforward_Update(MOTOR_DATA *motor, const Feedforward_Params_t *params);

#endif // FEEDFORWARD_H
