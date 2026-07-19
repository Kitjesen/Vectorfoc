#include "position_sensor_internal.h"

#include "board_config.h"

#include <stdint.h>

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109 &&                   \
    (!defined(TEST_ENV) || defined(POSITION_SENSOR_TEST_USE_DRIVERS))
#include "tmr3109_encoder.h"
#define POSITION_SENSOR_TMR3109_INIT_AVAILABLE 1
#else
#define POSITION_SENSOR_TMR3109_INIT_AVAILABLE 0
#endif

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109 && !defined(TEST_ENV)
#include "config.h"

#include <math.h>
#include <string.h>

#define POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE 1
#define POSITION_SENSOR_TWO_PI 6.28318530717959f
#else
#define POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE 0
#endif

static PositionSensorStatus_t PositionSensorTmr3109_Init(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
#if POSITION_SENSOR_TMR3109_INIT_AVAILABLE
  TMR3109_Init(&tmr3109_encoder_data, &HW_ENC_SPI, HW_ENC_CS_PORT,
               HW_ENC_CS_PIN);
#endif
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_UpdateAndRead(PositionSensorSample_t *sample) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  if (TMR3109_Update(&tmr3109_encoder_data, CURRENT_MEASURE_PERIOD) !=
      TMR3109_OK) {
    return POSITION_SENSOR_STATUS_IO_ERROR;
  }
  sample->position_rad =
      tmr3109_encoder_data.pos_estimate_ * POSITION_SENSOR_TWO_PI;
  sample->mechanical_angle_rad = tmr3109_encoder_data.mec_angle_rad;
  sample->velocity_rad_s = tmr3109_encoder_data.velocity_rad_s;
  sample->electrical_angle_rad = tmr3109_encoder_data.elec_angle_rad;
  sample->native_raw = (int32_t)tmr3109_encoder_data.raw_angle;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)sample;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_GetElectricalOffset(float *offset_rad) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  uint8_t pole_pairs = tmr3109_encoder_data.pole_pairs == 0u
                           ? 1u
                           : tmr3109_encoder_data.pole_pairs;
  *offset_rad = ((float)tmr3109_encoder_data.offset_counts *
                 POSITION_SENSOR_TWO_PI * (float)pole_pairs) /
                TMR3109_CPR_F;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_SetElectricalOffset(float offset_rad) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  uint8_t pole_pairs = tmr3109_encoder_data.pole_pairs == 0u
                           ? 1u
                           : tmr3109_encoder_data.pole_pairs;
  float mechanical_turns =
      offset_rad / (POSITION_SENSOR_TWO_PI * (float)pole_pairs);
  tmr3109_encoder_data.offset_rev = mechanical_turns;
  tmr3109_encoder_data.offset_counts =
      (int32_t)lroundf(mechanical_turns * TMR3109_CPR_F);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_SetPolePairs(uint8_t pole_pairs) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  tmr3109_encoder_data.pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)pole_pairs;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_ZeroMechanical(void) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  TMR3109_ResetCount(&tmr3109_encoder_data);
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_GetHealth(PositionSensorDriverHealth_t *health) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  uint64_t errors = (uint64_t)tmr3109_encoder_data.spi_err_count +
                    tmr3109_encoder_data.crc_err_count +
                    tmr3109_encoder_data.chip_err_count;
  health->valid = tmr3109_encoder_data.last_status == TMR3109_OK;
  health->diagnostic_flags = 0u;
  if (tmr3109_encoder_data.last_status == TMR3109_ERR_SPI) {
    health->diagnostic_flags |= POSITION_SENSOR_DIAGNOSTIC_TRANSPORT;
  } else if (tmr3109_encoder_data.last_status == TMR3109_ERR_CRC) {
    health->diagnostic_flags |= POSITION_SENSOR_DIAGNOSTIC_FRAME_CHECK;
  } else if (tmr3109_encoder_data.last_status == TMR3109_ERR_CHIP) {
    health->diagnostic_flags |= POSITION_SENSOR_DIAGNOSTIC_DEVICE;
  }
  health->transport_error_score =
      errors > UINT32_MAX ? UINT32_MAX : (uint32_t)errors;
  health->calibrated = tmr3109_encoder_data.calib_valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)health;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_CaptureCalibration(
    PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  snapshot->valid = tmr3109_encoder_data.calib_valid;
  memcpy(snapshot->offset_lut, tmr3109_encoder_data.offset_lut,
         sizeof(snapshot->offset_lut));
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_RestoreCalibration(
    const PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  memcpy(tmr3109_encoder_data.offset_lut, snapshot->offset_lut,
         sizeof(tmr3109_encoder_data.offset_lut));
  tmr3109_encoder_data.calib_valid = snapshot->valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_ClearCalibration(void) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  memset(tmr3109_encoder_data.offset_lut, 0,
         sizeof(tmr3109_encoder_data.offset_lut));
  tmr3109_encoder_data.calib_valid = false;
  tmr3109_encoder_data.offset_counts = 0;
  tmr3109_encoder_data.offset_rev = 0.0f;
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_SetCalibrationValid(
    bool valid) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  tmr3109_encoder_data.calib_valid = valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)valid;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_RawRead(PositionSensorRawCalibrationState_t *state) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  state->shadow_count = tmr3109_encoder_data.shadow_count;
  state->count_in_cpr = tmr3109_encoder_data.count_in_cpr;
  state->cpr = TMR3109_CPR;
  state->pole_pairs = tmr3109_encoder_data.pole_pairs;
  state->direction = tmr3109_encoder_data.dir == TMR3109_DIR_CW
                         ? POSITION_SENSOR_DIRECTION_CLOCKWISE
                         : POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE;
  state->offset_counts = tmr3109_encoder_data.offset_counts;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)state;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_RawPrepareClockwise(void) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  tmr3109_encoder_data.dir = TMR3109_DIR_CW;
  TMR3109_RebaseTracking(&tmr3109_encoder_data);
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_RawSetDirectionAndRebase(
    PositionSensorDirection_t direction) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  tmr3109_encoder_data.dir = direction == POSITION_SENSOR_DIRECTION_CLOCKWISE
                                 ? TMR3109_DIR_CW
                                 : TMR3109_DIR_CCW;
  TMR3109_RebaseTracking(&tmr3109_encoder_data);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)direction;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorTmr3109_RawCommitPolePairs(uint8_t pole_pairs) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  tmr3109_encoder_data.pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)pole_pairs;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorTmr3109_RawCommitOffsetAndLut(
    int32_t offset_counts,
    const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]) {
#if POSITION_SENSOR_TMR3109_RUNTIME_AVAILABLE
  tmr3109_encoder_data.offset_counts = offset_counts;
  tmr3109_encoder_data.offset_rev = (float)offset_counts / TMR3109_CPR_F;
  memcpy(tmr3109_encoder_data.offset_lut, offset_lut,
         sizeof(tmr3109_encoder_data.offset_lut));
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_counts;
  (void)offset_lut;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

const PositionSensorAdapter_t *PositionSensorTmr3109_GetAdapter(void) {
  static const PositionSensorAdapter_t adapter = {
      .descriptor =
          {
              .name = "TMR3109",
              .capabilities = POSITION_SENSOR_CAP_ABSOLUTE |
                              POSITION_SENSOR_CAP_HEALTH |
                              POSITION_SENSOR_CAP_RAW_DIRECTION_POLE |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION |
                              POSITION_SENSOR_CAP_LINEARITY_CALIBRATION,
          },
      .runtime =
          {
              .init = PositionSensorTmr3109_Init,
              .update_and_read = PositionSensorTmr3109_UpdateAndRead,
              .set_pole_pairs = PositionSensorTmr3109_SetPolePairs,
              .zero_mechanical_position = PositionSensorTmr3109_ZeroMechanical,
              .set_electrical_offset =
                  PositionSensorTmr3109_SetElectricalOffset,
              .get_electrical_offset =
                  PositionSensorTmr3109_GetElectricalOffset,
              .get_health = PositionSensorTmr3109_GetHealth,
          },
      .calibration =
          {
              .capture = PositionSensorTmr3109_CaptureCalibration,
              .restore = PositionSensorTmr3109_RestoreCalibration,
              .clear = PositionSensorTmr3109_ClearCalibration,
              .set_valid = PositionSensorTmr3109_SetCalibrationValid,
          },
      .raw_calibration =
          {
              .read = PositionSensorTmr3109_RawRead,
              .prepare_clockwise = PositionSensorTmr3109_RawPrepareClockwise,
              .set_direction_and_rebase =
                  PositionSensorTmr3109_RawSetDirectionAndRebase,
              .commit_pole_pairs = PositionSensorTmr3109_RawCommitPolePairs,
              .commit_offset_and_lut =
                  PositionSensorTmr3109_RawCommitOffsetAndLut,
          },
  };
  return &adapter;
}
