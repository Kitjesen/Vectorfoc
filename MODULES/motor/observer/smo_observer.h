#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

#include "observer.h"

// SMO specific parameters and state
typedef struct {
  float alpha;
  float beta;
  // State variables
  float est_i_alpha;
  float est_i_beta;
  float est_bemf_alpha;
  float est_bemf_beta;
  float est_angle;
  float est_velocity;
  float pll_angle;
  float pll_velocity;
  float pll_kp;
  float pll_ki;
} SMO_Observer_t;

void SMO_Observer_Init(SMO_Observer_t *smo);
void SMO_Observer_Update(void *pMemory, MOTOR_DATA *motor);

#endif // SMO_OBSERVER_H
