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

/**
 * @file calib_encoder.c
 * @brief Sensor-independent direction, pole-pair, and linearity calibration
 */

#include "calib_encoder.h"

#include "control/impl.h"
#include "hal_pwm.h"
#include "position_sensor.h"

#if !defined(TEST_ENV)
#include "hal_encoder.h"
#include "param_access.h"
#endif

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define OFFSET_LUT_NUM POSITION_SENSOR_CALIBRATION_LUT_SIZE
#define MIN_MOVEMENT_COUNTS 500u

static bool EncoderCalib_HasCapability(PositionSensorCapabilities_t capability) {
  const PositionSensorDescriptor_t *descriptor =
      PositionSensor_GetDescriptor();
  return descriptor != NULL &&
         (descriptor->capabilities & capability) == capability;
}

static int EncoderCalib_WrappedError(int32_t measured, int32_t reference,
                                     uint32_t cpr) {
  int32_t error;
  if (cpr == 0u) {
    return 0;
  }
  error = measured - reference;
  error %= (int32_t)cpr;
  if (error < 0) {
    error += (int32_t)cpr;
  }
  return (int)error;
}

static int16_t EncoderCalib_ClampLutValue(int64_t value) {
  if (value > INT16_MAX) {
    return INT16_MAX;
  }
  if (value < INT16_MIN) {
    return INT16_MIN;
  }
  return (int16_t)value;
}

static void EncoderCalib_ClearPidState(MOTOR_DATA *motor) {
  PID_clear(&motor->IqPID);
  PID_clear(&motor->IdPID);
  PID_clear(&motor->VelPID);
  PID_clear(&motor->PosPID);
}

static void EncoderCalib_PersistSuccess(MOTOR_DATA *motor) {
  if (motor == NULL) {
    return;
  }
#if !defined(TEST_ENV)
  g_add_offset = MHAL_Encoder_GetOffset();
  (void)Param_WriteFloat(PARAM_ADD_OFFSET, g_add_offset);
  Param_ScheduleSave();
#else
  (void)motor;
#endif
}

CalibResult DirectionPoleCalib_Update(MOTOR_DATA *motor,
                                      DirectionPoleCalibContext *ctx) {
  PositionSensorRawCalibrationState_t raw;
  PositionSensorStatus_t status;
  float time;

  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  if (!EncoderCalib_HasCapability(
          POSITION_SENSOR_CAP_RAW_DIRECTION_POLE)) {
    if (PositionSensor_SetCalibrationValid(true) != POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    motor->state.Cs_State = CS_ENCODER_START;
    return CALIB_SUCCESS;
  }

  time = (float)ctx->loop_count * CURRENT_MEASURE_PERIOD;
  switch (motor->state.Cs_State) {
  case CS_DIR_PP_START:
    if (ctx->loop_count == 0u) {
      ctx->phase_set = 0.0f;
      ctx->voltage = CURRENT_MAX_CALIB * motor->parameters.Rs * 3.0f / 2.0f;
    }
    Control_InjectVoltage(motor, ctx->voltage * time / 2.0f, 0.0f,
                          ctx->phase_set);
    if (time >= 2.0f) {
      status = PositionSensor_RawCalibrationRead(&raw);
      if (status != POSITION_SENSOR_STATUS_OK || raw.cpr == 0u) {
        return CALIB_FAILED_INVALID_PARAMS;
      }
      ctx->start_count = raw.shadow_count;
      motor->state.Cs_State = CS_DIR_PP_LOOP;
    }
    ctx->loop_count++;
    return CALIB_IN_PROGRESS;

  case CS_DIR_PP_LOOP:
    ctx->phase_set += CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
    Control_InjectVoltage(motor, ctx->voltage, 0.0f, ctx->phase_set);
    if (ctx->phase_set >= 4.0f * M_2PI) {
      motor->state.Cs_State = CS_DIR_PP_END;
    }
    ctx->loop_count++;
    return CALIB_IN_PROGRESS;

  case CS_DIR_PP_END: {
    int64_t diff_wide;
    int32_t diff;
    uint32_t abs_diff;
    float exact_pp;
    float nearest_pp;
    uint8_t estimated_pp;
    PositionSensorDirection_t direction;

    status = PositionSensor_RawCalibrationRead(&raw);
    if (status != POSITION_SENSOR_STATUS_OK || raw.cpr == 0u) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    diff_wide = raw.shadow_count - ctx->start_count;
    if (diff_wide > INT32_MAX || diff_wide < INT32_MIN) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    diff = (int32_t)diff_wide;
    abs_diff = diff < 0 ? (uint32_t)(-(int64_t)diff) : (uint32_t)diff;
    if (abs_diff < MIN_MOVEMENT_COUNTS) {
      return CALIB_FAILED_NO_MOVEMENT;
    }

    direction = diff > 0 ? POSITION_SENSOR_DIRECTION_CLOCKWISE
                         : POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE;
    status =
        PositionSensor_RawCalibrationSetDirectionAndRebase(direction);
    if (status != POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }

    exact_pp = (4.0f * (float)raw.cpr) / (float)abs_diff;
    if (exact_pp < 0.5f || exact_pp > (float)MAX_POLE_PAIRS) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    nearest_pp = roundf(exact_pp);
    if (fabsf(exact_pp - nearest_pp) > 0.15f) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    estimated_pp = (uint8_t)nearest_pp;
    if (estimated_pp == 0u) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    status = PositionSensor_RawCalibrationCommitPolePairs(estimated_pp);
    if (status != POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }

    /* MOTOR_PARAMETERS is the ISR's source of truth; update it atomically with
     * the sensor adapter so the next control cycle cannot restore the old PP. */
    motor->parameters.pole_pairs = (int)estimated_pp;
    motor->params_updated = true;
    motor->state.Cs_State = CS_ENCODER_START;
    return CALIB_SUCCESS;
  }

  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}

CalibResult EncoderCalib_Update(MOTOR_DATA *motor, EncoderCalibContext *ctx) {
  PositionSensorRawCalibrationState_t raw;
  PositionSensorStatus_t status;
  float time;
  float voltage;

  if (motor == NULL || ctx == NULL) {
    return CALIB_FAILED_INVALID_PARAMS;
  }

  if (!EncoderCalib_HasCapability(
          POSITION_SENSOR_CAP_LINEARITY_CALIBRATION)) {
    if (PositionSensor_SetCalibrationValid(true) != POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    EncoderCalib_PersistSuccess(motor);
    EncoderCalib_ClearPidState(motor);
    return CALIB_SUCCESS;
  }

  time = (float)ctx->loop_count * CURRENT_MEASURE_PERIOD;
  voltage = CURRENT_MAX_CALIB * motor->parameters.Rs * 3.0f / 2.0f;
  switch (motor->state.Cs_State) {
  case CS_ENCODER_START:
    ctx->phase_set = 0.0f;
    ctx->loop_count = 0u;
    ctx->sample_count = 0;
    ctx->next_sample_time = 0.0f;
    if (ctx->error_array == NULL || ctx->error_array_size == 0u ||
        ctx->offset_lut == NULL || ctx->offset_lut_size < OFFSET_LUT_NUM) {
      return CALIB_FAILED_MEMORY;
    }
    status = PositionSensor_RawCalibrationRead(&raw);
    if (status != POSITION_SENSOR_STATUS_OK || raw.cpr == 0u ||
        raw.pole_pairs == 0u ||
        (size_t)raw.pole_pairs * SAMPLES_PER_POLE_PAIR >
            ctx->error_array_size) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    if (PositionSensor_SetCalibrationValid(false) !=
        POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    memset(ctx->error_array, 0, ctx->error_array_size * sizeof(int));
    memset(ctx->offset_lut, 0, ctx->offset_lut_size * sizeof(int16_t));
    motor->state.Cs_State = CS_ENCODER_CW_LOOP;
    return CALIB_IN_PROGRESS;

  case CS_ENCODER_CW_LOOP: {
    int total_samples;
    status = PositionSensor_RawCalibrationRead(&raw);
    if (status != POSITION_SENSOR_STATUS_OK || raw.cpr == 0u ||
        raw.pole_pairs == 0u) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    total_samples = (int)raw.pole_pairs * SAMPLES_PER_POLE_PAIR;
    if ((size_t)total_samples > ctx->error_array_size) {
      return CALIB_FAILED_MEMORY;
    }
    if (ctx->sample_count < total_samples) {
      if (time > ctx->next_sample_time) {
        int32_t count_ref;
        ctx->next_sample_time +=
            M_2PI / ((float)SAMPLES_PER_POLE_PAIR * CALIB_PHASE_VEL);
        count_ref = (int32_t)((ctx->phase_set * (float)raw.cpr) /
                              (M_2PI * (float)raw.pole_pairs));
        ctx->error_array[ctx->sample_count] = EncoderCalib_WrappedError(
            raw.count_in_cpr, count_ref, raw.cpr);
        ctx->sample_count++;
      }
      ctx->phase_set += CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
    } else {
      ctx->phase_set -= CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
      ctx->loop_count = 0u;
      ctx->sample_count--;
      ctx->next_sample_time = 0.0f;
      motor->state.Cs_State = CS_ENCODER_CCW_LOOP;
    }
    Control_InjectVoltage(motor, voltage, 0.0f, ctx->phase_set);
    ctx->loop_count++;
    return CALIB_IN_PROGRESS;
  }

  case CS_ENCODER_CCW_LOOP:
    status = PositionSensor_RawCalibrationRead(&raw);
    if (status != POSITION_SENSOR_STATUS_OK || raw.cpr == 0u ||
        raw.pole_pairs == 0u) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    if (ctx->sample_count >= 0) {
      if (time > ctx->next_sample_time) {
        int32_t count_ref;
        int error;
        if ((size_t)ctx->sample_count >= ctx->error_array_size) {
          return CALIB_FAILED_MEMORY;
        }
        ctx->next_sample_time +=
            M_2PI / ((float)SAMPLES_PER_POLE_PAIR * CALIB_PHASE_VEL);
        count_ref = (int32_t)((ctx->phase_set * (float)raw.cpr) /
                              (M_2PI * (float)raw.pole_pairs));
        error = EncoderCalib_WrappedError(raw.count_in_cpr, count_ref, raw.cpr);
        ctx->error_array[ctx->sample_count] =
            (ctx->error_array[ctx->sample_count] + error) / 2;
        ctx->sample_count--;
      }
      ctx->phase_set -= CALIB_PHASE_VEL * CURRENT_MEASURE_PERIOD;
    } else {
      (void)MHAL_PWM_Brake();
      motor->state.Cs_State = CS_ENCODER_END;
    }
    Control_InjectVoltage(motor, voltage, 0.0f, ctx->phase_set);
    ctx->loop_count++;
    return CALIB_IN_PROGRESS;

  case CS_ENCODER_END: {
    int64_t moving_avg = 0;
    int32_t offset_counts;
    int total_samples;
    int window = SAMPLES_PER_POLE_PAIR;
    int lut_offset;

    status = PositionSensor_RawCalibrationRead(&raw);
    if (status != POSITION_SENSOR_STATUS_OK || raw.cpr == 0u ||
        raw.pole_pairs == 0u) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    total_samples = (int)raw.pole_pairs * SAMPLES_PER_POLE_PAIR;
    if (total_samples <= 0 || (size_t)total_samples > ctx->error_array_size ||
        ctx->offset_lut == NULL || ctx->offset_lut_size < OFFSET_LUT_NUM) {
      return CALIB_FAILED_MEMORY;
    }
    for (int i = 0; i < total_samples; ++i) {
      moving_avg += ctx->error_array[i];
    }
    offset_counts = (int32_t)(moving_avg / total_samples);
    lut_offset =
        (ctx->error_array[0] * (int)OFFSET_LUT_NUM) / (int32_t)raw.cpr;

    for (int i = 0; i < (int)OFFSET_LUT_NUM; ++i) {
      int lut_index;
      moving_avg = 0;
      for (int j = -window / 2; j < window / 2; ++j) {
        int index = i * total_samples / (int)OFFSET_LUT_NUM + j;
        if (index < 0) {
          index += total_samples;
        } else if (index >= total_samples) {
          index -= total_samples;
        }
        moving_avg += ctx->error_array[index];
      }
      moving_avg /= window;
      lut_index = (lut_offset + i) % (int)OFFSET_LUT_NUM;
      ctx->offset_lut[lut_index] =
          EncoderCalib_ClampLutValue(moving_avg - offset_counts);
    }

    status = PositionSensor_RawCalibrationCommitOffsetAndLut(
        offset_counts, ctx->offset_lut);
    if (status != POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    motor->state.Cs_State = CS_REPORT_OFFSET_LUT;
    ctx->loop_count = 0u;
    ctx->sample_count = 0;
    ctx->next_sample_time = 0.0f;
    return CALIB_IN_PROGRESS;
  }

  case CS_REPORT_OFFSET_LUT:
    if (ctx->sample_count < (int)OFFSET_LUT_NUM) {
      if (time > ctx->next_sample_time) {
        ctx->next_sample_time += 0.001f;
        ctx->sample_count++;
      }
      ctx->loop_count++;
      return CALIB_IN_PROGRESS;
    }
    if (PositionSensor_SetCalibrationValid(true) != POSITION_SENSOR_STATUS_OK) {
      return CALIB_FAILED_INVALID_PARAMS;
    }
    EncoderCalib_PersistSuccess(motor);
    EncoderCalib_ClearPidState(motor);
    return CALIB_SUCCESS;

  default:
    return CALIB_FAILED_INVALID_PARAMS;
  }
}
