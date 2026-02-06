#include "calib_encoder.h"
#include "control/impl.h" // For Control_InjectVoltage
#include "hal_pwm.h" // For MHAL_PWM_Brake (统一 HAL)
#include "mt6816_encoder.h"           // Include specific encoder header

#include "config.h" // For calibration constants
#include <math.h>
#include <string.h>

// Helper macro to access concrete encoder
#define ENC ((MT6816_Handle_t *)motor->components.encoder)
#define CW MT6816_DIR_CW
#define CCW MT6816_DIR_CCW
#define ENCODER_CPR_F MT6816_CPR_F
#define ENCODER_CPR MT6816_CPR
#define OFFSET_LUT_NUM 128

/**
 * @file calib_encoder.c
 * @brief Encoder, pole pair, and direction calibration implementation
 */

//=============================================================================
// Direction and Pole Pair Calibration
//=============================================================================

CalibResult DirectionPoleCalib_Update(MOTOR_DATA *motor,
                                      DirectionPoleCalibContext *ctx) {
  if (motor == NULL || ctx == NULL)
    return CALIB_FAILED_INVALID_PARAMS;

  float time = (float)ctx->loop_count * CURRENT_MEASURE_PERIOD;

  switch (motor->state.Cs_State) {
  case CS_DIR_PP_START:
    if (ctx->loop_count == 0) {
      ctx->phase_set = 0;
      ctx->voltage = CURRENT_MAX_CALIB * motor->parameters.Rs * 3.0f / 2.0f;
    }

    // Slowly ramp up voltage to target
    Control_InjectVoltage(motor, (ctx->voltage * time / 2.0f), 0.0f,
                ctx->phase_set);

    if (time >= 2.0f) {
      ctx->start_count = (float)ENC->shadow_count;
      motor->state.Cs_State = CS_DIR_PP_LOOP;
    }

    ctx->loop_count++;
    return CALIB_IN_PROGRESS;

  case CS_DIR_PP_LOOP:
    ctx->phase_set += CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
    Control_InjectVoltage(motor, ctx->voltage, 0.0f, ctx->phase_set);

    // Rotate 4 electrical cycles
    if (ctx->phase_set >= 4.0f * M_2PI) {
      motor->state.Cs_State = CS_DIR_PP_END;
    }

    ctx->loop_count++;
    return CALIB_IN_PROGRESS;

  case CS_DIR_PP_END: {
    int32_t diff = ENC->shadow_count - (int32_t)ctx->start_count;

    // Detect direction
    if (diff > 0) {
      ENC->dir = CW;
    } else {
      ENC->dir = CCW;
    }

// --- Pole Pair Identification (Robust) ---

// 1. Stall Detection / Minimal Movement Check
// Threshold: 10% of one revolution for expected minimal PP (approx 1 PP)
// 16384 * 0.1 = ~1600 counts. Let's be generous and say 500 counts.
#define MIN_MOVEMENT_COUNTS 500
    float abs_diff = (float)ABS(diff);

    if (abs_diff < MIN_MOVEMENT_COUNTS) {
      return CALIB_FAILED_NO_MOVEMENT;
    }

    // 2. Calculate Exact Pole Pairs
    // Formula: PP = (N_el_cycles * CPR) / Total_Counts
    // N_el_cycles = 4.0f
    float exact_pp = (4.0f * ENCODER_CPR_F) / abs_diff;

    // 3. Range Sanity Check
    // Valid range: 1 to MAX_POLE_PAIRS (usually 50 or 100)
    if (exact_pp < 0.5f || exact_pp > (float)MAX_POLE_PAIRS) {
      return CALIB_FAILED_INVALID_PARAMS;
    }

    // 4. Integer Confidence Check
    // The calculated PP should be very close to an integer.
    // Allow max deviation, e.g., 15% (0.15)
    float nearest_pp = roundf(exact_pp);
    float deviation = fabsf(exact_pp - nearest_pp);

    if (deviation > 0.15f) {
      // Deviation too high, indicates slippage, noise, or wrong CPR
      // We could return a specific error, for now INVALID_PARAMS
      return CALIB_FAILED_INVALID_PARAMS;
    }

    // 5. Final Assignment
    uint32_t estimated_pp = (uint32_t)nearest_pp;

    if (estimated_pp == 0)
      estimated_pp = 1; // Should be caught by range check, but safety first
    ENC->pole_pairs = estimated_pp;

    // Next step: Encoder Calibration
    motor->state.Cs_State = CS_ENCODER_START;

    return CALIB_SUCCESS;
  }

  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}

//=============================================================================
// Encoder offset and lookup table calibration
//=============================================================================

CalibResult EncoderCalib_Update(MOTOR_DATA *motor, EncoderCalibContext *ctx) {
  if (motor == NULL || ctx == NULL)
    return CALIB_FAILED_INVALID_PARAMS;

  float time = (float)ctx->loop_count * CURRENT_MEASURE_PERIOD;
  float voltage = CURRENT_MAX_CALIB * motor->parameters.Rs * 3.0f / 2.0f;

  switch (motor->state.Cs_State) {
  case CS_ENCODER_START:
    ctx->phase_set = 0;
    ctx->loop_count = 0;
    ctx->sample_count = 0;
    ctx->next_sample_time = 0;

    if (ctx->error_array == NULL || ctx->error_array_size == 0) {
      return CALIB_FAILED_MEMORY;
    }
    memset(ctx->error_array, 0, ctx->error_array_size * sizeof(int));

    motor->state.Cs_State = CS_ENCODER_CW_LOOP;
    return CALIB_IN_PROGRESS;

  case CS_ENCODER_CW_LOOP: {
    int total_samples = ENC->pole_pairs * SAMPLES_PER_POLE_PAIR;

    if (ctx->sample_count < total_samples) {
      if (time > ctx->next_sample_time) {
        ctx->next_sample_time +=
            M_2PI / ((float)SAMPLES_PER_POLE_PAIR * CALIB_PHASE_VEL);

        int count_ref = (int)((ctx->phase_set * ENCODER_CPR_F) /
                              (M_2PI * (float)ENC->pole_pairs));
        int error = ENC->count_in_cpr - count_ref;
        error += ENCODER_CPR * (error < 0);

        if (ctx->sample_count < ctx->error_array_size) {
          ctx->error_array[ctx->sample_count] = error;
        }

        ctx->sample_count++;
      }

      ctx->phase_set += CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
    } else {
      ctx->phase_set -= CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
      ctx->loop_count = 0;
      ctx->sample_count--;
      ctx->next_sample_time = 0;
      motor->state.Cs_State = CS_ENCODER_CCW_LOOP;
    }

    Control_InjectVoltage(motor, voltage, 0, ctx->phase_set);
    ctx->loop_count++;
    return CALIB_IN_PROGRESS;
  }

  case CS_ENCODER_CCW_LOOP: {
    if (ctx->sample_count >= 0) {
      if (time > ctx->next_sample_time) {
        ctx->next_sample_time +=
            M_2PI / ((float)SAMPLES_PER_POLE_PAIR * CALIB_PHASE_VEL);

        int count_ref = (int)((ctx->phase_set * ENCODER_CPR_F) /
                              (M_2PI * (float)ENC->pole_pairs));
        int error = ENC->count_in_cpr - count_ref;
        error += ENCODER_CPR * (error < 0);

        if (ctx->sample_count < ctx->error_array_size) {
          ctx->error_array[ctx->sample_count] =
              (ctx->error_array[ctx->sample_count] + error) / 2;
        }

        ctx->sample_count--;
      }

      ctx->phase_set -= CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
    } else {
      MHAL_PWM_Brake();
      motor->state.Cs_State = CS_ENCODER_END;
    }

    Control_InjectVoltage(motor, voltage, 0, ctx->phase_set);
    ctx->loop_count++;
    return CALIB_IN_PROGRESS;
  }

  case CS_ENCODER_END: {
    // Calculate average offset
    int64_t moving_avg = 0;
    int total_samples = ENC->pole_pairs * SAMPLES_PER_POLE_PAIR;

    for (int i = 0; i < total_samples; i++) {
      if (i < ctx->error_array_size)
        moving_avg += ctx->error_array[i];
    }
    ENC->offset_counts = (int)(moving_avg / total_samples);

    // Generate lookup table (FIR filter)
    int window = SAMPLES_PER_POLE_PAIR;
    int lut_offset = 0;
    if (ctx->error_array_size > 0)
      lut_offset = ctx->error_array[0] * OFFSET_LUT_NUM / ENCODER_CPR;

    for (int i = 0; i < OFFSET_LUT_NUM; i++) {
      moving_avg = 0;
      for (int j = (-window) / 2; j < (window) / 2; j++) {
        int index =
            i * ENC->pole_pairs * SAMPLES_PER_POLE_PAIR / OFFSET_LUT_NUM + j;

        // Boundary handling
        if (index < 0) {
          index += (SAMPLES_PER_POLE_PAIR * ENC->pole_pairs);
        } else if (index > (SAMPLES_PER_POLE_PAIR * ENC->pole_pairs - 1)) {
          index -= (SAMPLES_PER_POLE_PAIR * ENC->pole_pairs);
        }

        if (index < ctx->error_array_size)
          moving_avg += ctx->error_array[index];
      }

      moving_avg = moving_avg / window;
      int lut_index = lut_offset + i;
      if (lut_index > (OFFSET_LUT_NUM - 1)) {
        lut_index -= OFFSET_LUT_NUM;
      }

      ENC->offset_lut[lut_index] = (int16_t)(moving_avg - ENC->offset_counts);
    }

    motor->state.Cs_State = CS_REPORT_OFFSET_LUT;
    ctx->loop_count = 0;
    ctx->sample_count = 0;
    ctx->next_sample_time = 0;
    return CALIB_IN_PROGRESS;
  }

  case CS_REPORT_OFFSET_LUT:
    if (ctx->sample_count < OFFSET_LUT_NUM) {
      if (time > ctx->next_sample_time) {
        ctx->next_sample_time += 0.001f;
        ctx->sample_count++;
      }
      ctx->loop_count++;
      return CALIB_IN_PROGRESS;
    } else {
      // Calibration fully completed
      ENC->calib_valid = true;

      // Clear PID state
      PID_clear(&motor->IqPID);
      PID_clear(&motor->IdPID);
      PID_clear(&motor->VelPID);
      PID_clear(&motor->PosPID);

      return CALIB_SUCCESS;
    }

  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}
