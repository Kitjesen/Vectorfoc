#ifndef TEST_ENCODER_INIT_MOTOR_H
#define TEST_ENCODER_INIT_MOTOR_H

#include "board_config.h"
#include "motor_hal_api.h"

typedef struct {
  const Motor_HAL_Handle_t *hal;
} MOTOR_COMPONENTS;

typedef struct {
  int pole_pairs;
} MOTOR_PARAMETERS;

typedef struct {
  MOTOR_COMPONENTS components;
  MOTOR_PARAMETERS parameters;
} MOTOR_DATA;

extern MOTOR_DATA motor_data;

#endif /* TEST_ENCODER_INIT_MOTOR_H */
