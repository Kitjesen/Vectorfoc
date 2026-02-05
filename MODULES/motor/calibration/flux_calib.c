#include "flux_calib.h"
#include "motor/control/modes/impl.h" // For Control_InjectVoltage
#include "motor/hal/motor_hal_api.h" // For MHAL_PWM_Brake
#include "motor/config.h"
#include "motor/foc/clarke.h"
#include "motor/foc/park.h"
#include "mt6816_encoder.h"
#include <math.h>

/**
 * @file flux_calib.c
 * @brief Flux calibration implementation (Back-EMF method)
 */

/**
 * @brief Initialize flux calibration
 */
CalibResult FluxCalib_Start(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  FluxCalibContext *flux = &ctx->flux;

  // Reset state
  flux->loop_count = 0;
  flux->phase_set = 0;
  flux->flux_sum = 0.0f;
  flux->flux_samples = 0;
  flux->target_velocity = FLUX_CALIB_VEL / M_2PI; // Convert to turn/s

  // Start motor rotation with small test voltage
  Control_InjectVoltage(motor, 0.0f, 1.0f, flux->phase_set);

  motor->state.Cs_State = CS_FLUX_LOOP;

  return CALIB_IN_PROGRESS;
}

/**
 * @brief Update flux calibration
 */
CalibResult FluxCalib_Update(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  if (motor->state.Cs_State != CS_FLUX_LOOP) {
    return FluxCalib_Start(motor, ctx);
  }

  FluxCalibContext *flux = &ctx->flux;
  uint32_t total_cycles = (uint32_t)(CURRENT_MEASURE_HZ * FLUX_CALIB_DURATION);

  // Slowly rotate motor
  flux->phase_set += FLUX_CALIB_VEL * CURRENT_MEASURE_PERIOD;
  Control_InjectVoltage(motor, 0.0f, 1.5f, flux->phase_set);

  MT6816_Handle_t *enc = (MT6816_Handle_t *)motor->components.encoder;

  // Wait for speed to stabilize before sampling (Improved algorithm)
  if (flux->loop_count > CURRENT_MEASURE_HZ * 1.5f && // Wait 1.5s
      enc->vel_estimate_ > 0.05f)                     // Ensure sufficient speed
  {
    // Check speed stability
    float vel_error = fabsf(enc->vel_estimate_ - flux->target_velocity);

    if (vel_error < 0.02f) { // Stable within 0.02 turn/s
      // Calculate Back-EMF: E = V - I*R (Ignore L*dI/dt at low speed)
      float Ialpha = 0.0f;
      float Ibeta = 0.0f;
      float Id = 0.0f;
      float Iq = 0.0f;
      Clarke_Transform(motor->algo_input.Ia, motor->algo_input.Ib,
                       motor->algo_input.Ic, &Ialpha, &Ibeta);
      Park_Transform(Ialpha, Ibeta, motor->feedback.phase_angle, &Id, &Iq);

      float back_emf_q = motor->algo_output.Vq - Iq * motor->parameters.Rs;

      // Flux = Back-EMF / Electrical Angular Velocity
      float omega_e = enc->vel_estimate_ * (float)enc->pole_pairs * M_2PI;

      // Division by zero protection
      if (fabsf(omega_e) < 1e-3f) {
        omega_e = 1e-3f;
      }

      float flux_instantaneous = fabsf(back_emf_q) / omega_e;

      // Use stricter flux range check
      if (flux_instantaneous > FLUX_VALID_MIN &&
          flux_instantaneous < FLUX_VALID_MAX) {
        flux->flux_sum += flux_instantaneous;
        flux->flux_samples++;
      }
    }
  }

  flux->loop_count++;

  // Check if complete
  if (flux->loop_count >= total_cycles) {
    return FluxCalib_Finish(motor, ctx);
  }

  return CALIB_IN_PROGRESS;
}

/**
 * @brief Finish flux calibration
 */
CalibResult FluxCalib_Finish(MOTOR_DATA *motor, CalibrationContext *ctx) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  FluxCalibContext *flux = &ctx->flux;

  // Calculate average flux
  if (flux->flux_samples > 100) { // Ensure sufficient samples
    motor->parameters.flux = flux->flux_sum / (float)flux->flux_samples;

    // Update Torque Constant: Kt = 1.5 * Pp * Flux [Nm/A]
    motor->Controller.torque_const =
        1.5f * (float)motor->parameters.pole_pairs * motor->parameters.flux;
  } else {
    // Insufficient samples, use default
    motor->parameters.flux = 0.01f;
    motor->Controller.torque_const =
        1.5f * (float)motor->parameters.pole_pairs * motor->parameters.flux;

    // Optional: return warning
    // return CALIB_FAILED_INSUFFICIENT_DATA;
  }

  // Disable PWM
  MHAL_PWM_Brake();

  // Reset state
  flux->flux_sum = 0.0f;
  flux->flux_samples = 0;
  flux->loop_count = 0;

  // Switch to running state
  motor->state.Cs_State = CS_FLUX_END;
  motor->state.Sub_State = SUB_STATE_IDLE;
  motor->state.State_Mode = STATE_MODE_RUNNING;
  MT6816_Handle_t *enc = (MT6816_Handle_t *)motor->components.encoder;
  enc->calib_valid = true;

  // Clear PID state
  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);

  return CALIB_SUCCESS;
}
