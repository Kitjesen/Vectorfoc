// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flux_calib.h"
#include "control/impl.h"
#include "hal_pwm.h"
#include "config.h"
#include "foc/clarke.h"
#include "foc/park.h"
#include "hal_encoder.h"
#ifdef BOARD_XSTAR
#include "board_config_xstar.h"
#include "hall_encoder.h"
#include "abz_encoder.h"
#else
#include "mt6816_encoder.h"
#endif
#include <math.h>

/**
 * @file flux_calib.c
 * @brief Flux calibration implementation (Back-EMF method)
 */

CalibResult FluxCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  FluxCalibContext *flux = &ctx->flux;
  flux->loop_count = 0;
  flux->phase_set = 0;
  flux->flux_sum = 0.0f;
  flux->flux_samples = 0;
  flux->target_velocity = FLUX_CALIB_VEL / M_2PI;

  Control_InjectVoltage(motor, 0.0f, 1.0f, flux->phase_set);
  motor->state.Cs_State = CS_FLUX_LOOP;
  return CALIB_IN_PROGRESS;
}

CalibResult FluxCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }
  if (motor->state.Cs_State != CS_FLUX_LOOP) {
    return FluxCalib_Start(motor, ctx);
  }

  FluxCalibContext *flux = &ctx->flux;
  uint32_t total_cycles = (uint32_t)(CURRENT_MEASURE_HZ * FLUX_CALIB_DURATION);
  float vel_estimate_turn_s = MHAL_Encoder_GetVelocity() / M_2PI;

  flux->phase_set += FLUX_CALIB_VEL * CURRENT_MEASURE_PERIOD;
  Control_InjectVoltage(motor, 0.0f, 1.5f, flux->phase_set);

  if (flux->loop_count > CURRENT_MEASURE_HZ * 1.5f &&
      vel_estimate_turn_s > 0.05f) {
    float vel_error = fabsf(vel_estimate_turn_s - flux->target_velocity);
    if (vel_error < 0.02f) {
      float Ialpha = 0.0f;
      float Ibeta = 0.0f;
      float Id = 0.0f;
      float Iq = 0.0f;

      Clarke_Transform(motor->algo_input.Ia, motor->algo_input.Ib,
                       motor->algo_input.Ic, &Ialpha, &Ibeta);
      Park_Transform(Ialpha, Ibeta, motor->feedback.phase_angle, &Id, &Iq);

      {
        float back_emf_q = motor->algo_output.Vq - Iq * motor->parameters.Rs;
        float omega_e =
            vel_estimate_turn_s * (float)motor->parameters.pole_pairs * M_2PI;
        if (fabsf(omega_e) >= 1.0f) {
          float flux_instantaneous = fabsf(back_emf_q) / omega_e;
          if (flux_instantaneous > FLUX_VALID_MIN &&
              flux_instantaneous < FLUX_VALID_MAX) {
            flux->flux_sum += flux_instantaneous;
            flux->flux_samples++;
          }
        }
      }
    }
  }

  flux->loop_count++;
  if (flux->loop_count >= total_cycles) {
    return FluxCalib_Finish(motor, ctx);
  }
  return CALIB_IN_PROGRESS;
}

CalibResult FluxCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  FluxCalibContext *flux = &ctx->flux;
  if (flux->flux_samples > 100) {
    motor->parameters.flux = flux->flux_sum / (float)flux->flux_samples;
    motor->Controller.torque_const =
        1.5f * (float)motor->parameters.pole_pairs * motor->parameters.flux;
  } else {
    motor->parameters.flux = 0.01f;
    motor->Controller.torque_const =
        1.5f * (float)motor->parameters.pole_pairs * motor->parameters.flux;
  }

  MHAL_PWM_Brake();
  flux->flux_sum = 0.0f;
  flux->flux_samples = 0;
  flux->loop_count = 0;
  motor->state.Cs_State = CS_FLUX_END;
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.State_Mode = STATE_MODE_RUNNING;

#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  hall_data.calib_valid = true;
#else
  abz_data.calib_valid = true;
#endif
#else
  ((MT6816_Handle_t *)motor->components.encoder)->calib_valid = true;
#endif

  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);
  return CALIB_SUCCESS;
}
