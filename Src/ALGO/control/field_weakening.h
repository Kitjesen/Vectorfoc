#ifndef FIELD_WEAKENING_H
#define FIELD_WEAKENING_H

#include "motor.h"

typedef struct {
  float max_weakening_current;
  float start_velocity;
} FieldWeakening_Config_t;

void FieldWeakening_Update(MOTOR_DATA *motor,
                           const FieldWeakening_Config_t *cfg);

#endif // FIELD_WEAKENING_H
