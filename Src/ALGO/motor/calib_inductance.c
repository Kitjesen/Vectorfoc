#include "calib_inductance.h"
#include "config.h" // For MOTOR_LS
#include "control/impl.h" // For Control_InjectVoltage
#include "hal_pwm.h" // For MHAL_PWM_Brake (统一 HAL)
#include <math.h>
#include <string.h> // for memset


/**
 * @file calib_inductance.c
 * @brief Inductance calibration implementation
 */

CalibResult InductanceCalib_Update(MOTOR_DATA *motor,
                                   InductanceCalibContext *ctx, float dt) {
  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  switch (motor->state.Cs_State) {
  case CS_MOTOR_L_START:
    ctx->loop_count = 0;
    ctx->Ialphas[0] = 0.0f;
    ctx->Ialphas[1] = 0.0f;

    // Initialize voltages (if not initialized)
    if (ctx->voltages[0] == 0.0f && ctx->voltages[1] == 0.0f) {
      ctx->voltages[0] = -VOLTAGE_MAX_CALIB;
      ctx->voltages[1] = +VOLTAGE_MAX_CALIB;
    }

    Control_InjectVoltage(motor, ctx->voltages[0], 0.0f, 0.0f);
    motor->state.Cs_State = CS_MOTOR_L_LOOP;
    return CALIB_IN_PROGRESS;

  case CS_MOTOR_L_LOOP: {
    int i = ctx->loop_count & 1; // Alternate between 0 and 1

    // Apply test voltage
    Control_InjectVoltage(motor, ctx->voltages[i], 0.0f, 0.0f);

    // Record only final steady state current
    if (ctx->loop_count >= LS_CALIB_CYCLES - 5) {
      ctx->Ialphas[i] += motor->algo_input.Ia / 5.0f;
    }

    ctx->loop_count++;

    if (ctx->loop_count >= (LS_CALIB_CYCLES << 1)) {
      MHAL_PWM_Brake();
      motor->state.Cs_State = CS_MOTOR_L_END;
    }
    return CALIB_IN_PROGRESS;
  }

  case CS_MOTOR_L_END: {
    // Calculate current change rate (di/dt)
    float dI_by_dt = (ctx->Ialphas[1] - ctx->Ialphas[0]) / dt;

    // Division by zero protection
    if (fabsf(dI_by_dt) < 1e-3f) {
      motor->parameters.Ls = 0.00003f; // Use default value
    } else {
      float L = VOLTAGE_MAX_CALIB / dI_by_dt;
      motor->parameters.Ls = fabsf(L * 2.0f / 3.0f);
    }

    // Update current loop gains
    CurrentLoop_UpdateGain(motor);

    // Next step is direction/pole pair calibration
    motor->state.Cs_State = CS_DIR_PP_START;

    return CALIB_SUCCESS;
  }

  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}
