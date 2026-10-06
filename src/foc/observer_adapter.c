// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "observer_adapter.h"
#include "motor_configuration.h"

void SMO_Observer_Init(SMO_Observer_t *smo) {
  SMO_Observer_AlgorithmInit(smo);
}

void SMO_Observer_Update(SMO_Observer_t *smo, MOTOR_DATA *motor) {
  if (smo == NULL || motor == NULL) return;
  SMO_ObserverInput_t input = {
      .rs = motor->parameters.Rs,
      .ls = motor->parameters.Ls,
      .i_alpha = motor->algo_output.Ialpha,
      .i_beta = motor->algo_output.Ibeta,
      .v_alpha = motor->algo_output.Valpha,
      .v_beta = motor->algo_output.Vbeta,
      .encoder_velocity_turn_s = motor->feedback.velocity,
      .encoder_angle_rad = motor->feedback.phase_angle,
      .dt_s = CURRENT_SAMPLE_PERIOD_S,
  };
  SMO_ObserverOutput_t output;
  SMO_Observer_AlgorithmUpdate(smo, &input, &output);
  motor->feedback.observer_angle = output.angle_rad;
  motor->feedback.observer_velocity = output.velocity_rad_s;
}
