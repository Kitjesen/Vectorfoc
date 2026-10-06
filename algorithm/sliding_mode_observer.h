// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef ALGORITHM_SMO_OBSERVER_H
#define ALGORITHM_SMO_OBSERVER_H

#include <stdbool.h>

/* State for a sensorless sliding-mode observer and its PLL. */
typedef struct {
  float alpha;
  float beta;
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
} SMO_ObserverState_t;

/* Numeric observer inputs. Mechanical encoder velocity is turn/s; electrical
 * angle and the returned velocity are rad and rad/s. */
typedef struct {
  float rs;
  float ls;
  float i_alpha;
  float i_beta;
  float v_alpha;
  float v_beta;
  float encoder_velocity_turn_s;
  float encoder_angle_rad;
  float dt_s;
} SMO_ObserverInput_t;

typedef struct {
  float angle_rad;
  float velocity_rad_s;
} SMO_ObserverOutput_t;

void SMO_Observer_AlgorithmInit(SMO_ObserverState_t *state);
void SMO_Observer_AlgorithmUpdate(SMO_ObserverState_t *state,
                                   const SMO_ObserverInput_t *input,
                                   SMO_ObserverOutput_t *output);

#endif /* ALGORITHM_SMO_OBSERVER_H */
