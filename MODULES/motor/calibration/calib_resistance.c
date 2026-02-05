#include "calib_resistance.h"
#include "motor/config.h"
#include "motor/control/modes/impl.h" // For Control_InjectVoltage
#include "motor/hal/motor_hal_api.h" // For MHAL_PWM_Brake
#include <math.h>


/**
 * @file calib_resistance.c
 * @brief Resistance calibration implementation
 */

CalibResult ResistanceCalib_Update(MOTOR_DATA *motor,
                                   ResistanceCalibContext *ctx, float dt) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  switch (motor->state.Cs_State) {
  case CS_MOTOR_R_START:
    ctx->loop_count = 0;
    ctx->voltage = 0.0f;

    // Initialize parameters (use default if kI is not set)
    if (ctx->kI == 0.0f)
      ctx->kI = 2.0f;

    motor->state.Cs_State = CS_MOTOR_R_LOOP;
    return CALIB_IN_PROGRESS;

  case CS_MOTOR_R_LOOP:
    // PI controller drives current to target
    // error = target - measured
    // integral += Ki * dt * error
    ctx->voltage +=
        ctx->kI * dt * (CURRENT_MAX_CALIB - motor->algo_input.Ia);

    // Apply test voltage along Phase A (SVPWM angle=0)
    Control_InjectVoltage(motor, ctx->voltage, 0, 0);

    ctx->loop_count++;

    if (ctx->loop_count >= RS_CALIB_CYCLES) {
      MHAL_PWM_Brake();
      motor->state.Cs_State = CS_MOTOR_R_END;
    }
    return CALIB_IN_PROGRESS;

  case CS_MOTOR_R_END:
    // Calculate resistance: R = V / I * (2/3)
    // Prevent division by zero
    if (CURRENT_MAX_CALIB > 1e-4f) {
      motor->parameters.Rs = (ctx->voltage / CURRENT_MAX_CALIB) * (2.0f / 3.0f);
    } else {
      motor->parameters.Rs = 0.0f; // Alternatively set to default
    }

    // Next step is usually inductance calibration
    motor->state.Cs_State = CS_MOTOR_L_START;

    return CALIB_SUCCESS;

  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}
