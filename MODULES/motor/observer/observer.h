#ifndef OBSERVER_H
#define OBSERVER_H

#include "motor.h"

/**
 * @brief Observer generic interface
 */
typedef struct {
  void (*update)(void *pMemory, MOTOR_DATA *motor);
  void (*reset)(void *pMemory);
} Observer_Interface_t;

// Function prototypes for integration
void Observer_Init(MOTOR_DATA *motor);
void Observer_Update(MOTOR_DATA *motor);

#endif // OBSERVER_H
