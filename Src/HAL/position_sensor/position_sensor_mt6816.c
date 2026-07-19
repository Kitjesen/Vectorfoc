#include "position_sensor_internal.h"

#include "board_config.h"

#include <stdint.h>

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_MT6816 &&                    \
    (!defined(TEST_ENV) || defined(POSITION_SENSOR_TEST_USE_DRIVERS))
#include "mt6816_encoder.h"
#define POSITION_SENSOR_MT6816_INIT_AVAILABLE 1
#else
#define POSITION_SENSOR_MT6816_INIT_AVAILABLE 0
#endif

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_MT6816 && !defined(TEST_ENV)
#include "config.h"

#include <math.h>
#include <string.h>

#define POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE 1
#define POSITION_SENSOR_TWO_PI 6.28318530717959f
#else
#define POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE 0
#endif

static PositionSensorStatus_t PositionSensorMt6816_Init(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_MT6816
#if POSITION_SENSOR_MT6816_INIT_AVAILABLE
  MT6816_Init(&encoder_data, &HW_ENC_SPI, HW_ENC_CS_PORT, HW_ENC_CS_PIN);
#endif
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_UpdateAndRead(PositionSensorSample_t *sample) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  if (MT6816_Update(&encoder_data, CURRENT_MEASURE_PERIOD) != MT6816_OK) {
    return POSITION_SENSOR_STATUS_IO_ERROR;
  }
  sample->position_rad = encoder_data.pos_estimate_ * POSITION_SENSOR_TWO_PI;
  sample->mechanical_angle_rad = encoder_data.mec_angle_rad;
  sample->velocity_rad_s = encoder_data.velocity_rad_s;
  sample->electrical_angle_rad = encoder_data.elec_angle_rad;
  sample->native_raw = (int32_t)encoder_data.raw_angle;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)sample;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_GetElectricalOffset(float *offset_rad) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  uint8_t pole_pairs =
      encoder_data.pole_pairs == 0u ? 1u : encoder_data.pole_pairs;
  *offset_rad = ((float)encoder_data.offset_counts * POSITION_SENSOR_TWO_PI *
                 (float)pole_pairs) /
                MT6816_CPR_F;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_SetElectricalOffset(float offset_rad) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  uint8_t pole_pairs =
      encoder_data.pole_pairs == 0u ? 1u : encoder_data.pole_pairs;
  float mechanical_turns =
      offset_rad / (POSITION_SENSOR_TWO_PI * (float)pole_pairs);
  encoder_data.offset_rev = mechanical_turns;
  encoder_data.offset_counts =
      (int32_t)lroundf(mechanical_turns * MT6816_CPR_F);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_SetPolePairs(uint8_t pole_pairs) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  encoder_data.pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)pole_pairs;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_ZeroMechanical(void) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  MT6816_ResetCount(&encoder_data);
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_GetHealth(PositionSensorDriverHealth_t *health) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  uint64_t errors =
      (uint64_t)encoder_data.rx_err_count + encoder_data.check_err_count;
  health->valid = encoder_data.last_status == MT6816_OK;
  health->diagnostic_flags = 0u;
  if (encoder_data.last_status == MT6816_ERR_SPI) {
    health->diagnostic_flags |= POSITION_SENSOR_DIAGNOSTIC_TRANSPORT;
  } else if (encoder_data.last_status == MT6816_ERR_PARITY) {
    health->diagnostic_flags |= POSITION_SENSOR_DIAGNOSTIC_FRAME_CHECK;
  } else if (encoder_data.last_status == MT6816_ERR_MAG) {
    health->diagnostic_flags |= POSITION_SENSOR_DIAGNOSTIC_DEVICE;
  }
  health->transport_error_score =
      errors > UINT32_MAX ? UINT32_MAX : (uint32_t)errors;
  health->calibrated = encoder_data.calib_valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)health;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_CaptureCalibration(
    PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  snapshot->valid = encoder_data.calib_valid;
  memcpy(snapshot->offset_lut, encoder_data.offset_lut,
         sizeof(snapshot->offset_lut));
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_RestoreCalibration(
    const PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  memcpy(encoder_data.offset_lut, snapshot->offset_lut,
         sizeof(encoder_data.offset_lut));
  encoder_data.calib_valid = snapshot->valid;
  encoder_data.is_calibrated = snapshot->valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_ClearCalibration(void) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  memset(encoder_data.offset_lut, 0, sizeof(encoder_data.offset_lut));
  encoder_data.calib_valid = false;
  encoder_data.is_calibrated = false;
  encoder_data.offset_counts = 0;
  encoder_data.offset_rev = 0.0f;
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_SetCalibrationValid(
    bool valid) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  encoder_data.calib_valid = valid;
  encoder_data.is_calibrated = valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)valid;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_RawRead(PositionSensorRawCalibrationState_t *state) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  state->shadow_count = encoder_data.shadow_count;
  state->count_in_cpr = encoder_data.count_in_cpr;
  state->cpr = MT6816_CPR;
  state->pole_pairs = encoder_data.pole_pairs;
  state->direction = encoder_data.dir == MT6816_DIR_CW
                         ? POSITION_SENSOR_DIRECTION_CLOCKWISE
                         : POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE;
  state->offset_counts = encoder_data.offset_counts;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)state;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_RawPrepareClockwise(void) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  encoder_data.dir = MT6816_DIR_CW;
  MT6816_RebaseTracking(&encoder_data);
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_RawSetDirectionAndRebase(
    PositionSensorDirection_t direction) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  encoder_data.dir = direction == POSITION_SENSOR_DIRECTION_CLOCKWISE
                         ? MT6816_DIR_CW
                         : MT6816_DIR_CCW;
  MT6816_RebaseTracking(&encoder_data);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)direction;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorMt6816_RawCommitPolePairs(uint8_t pole_pairs) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  encoder_data.pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)pole_pairs;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorMt6816_RawCommitOffsetAndLut(
    int32_t offset_counts,
    const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]) {
#if POSITION_SENSOR_MT6816_RUNTIME_AVAILABLE
  encoder_data.offset_counts = offset_counts;
  encoder_data.offset_rev = (float)offset_counts / MT6816_CPR_F;
  memcpy(encoder_data.offset_lut, offset_lut, sizeof(encoder_data.offset_lut));
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_counts;
  (void)offset_lut;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

const PositionSensorAdapter_t *PositionSensorMt6816_GetAdapter(void) {
  static const PositionSensorAdapter_t adapter = {
      .descriptor =
          {
              .name = "MT6816",
              .capabilities = POSITION_SENSOR_CAP_ABSOLUTE |
                              POSITION_SENSOR_CAP_HEALTH |
                              POSITION_SENSOR_CAP_RAW_DIRECTION_POLE |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION |
                              POSITION_SENSOR_CAP_LINEARITY_CALIBRATION,
          },
      .runtime =
          {
              .init = PositionSensorMt6816_Init,
              .update_and_read = PositionSensorMt6816_UpdateAndRead,
              .set_pole_pairs = PositionSensorMt6816_SetPolePairs,
              .zero_mechanical_position = PositionSensorMt6816_ZeroMechanical,
              .set_electrical_offset = PositionSensorMt6816_SetElectricalOffset,
              .get_electrical_offset = PositionSensorMt6816_GetElectricalOffset,
              .get_health = PositionSensorMt6816_GetHealth,
          },
      .calibration =
          {
              .capture = PositionSensorMt6816_CaptureCalibration,
              .restore = PositionSensorMt6816_RestoreCalibration,
              .clear = PositionSensorMt6816_ClearCalibration,
              .set_valid = PositionSensorMt6816_SetCalibrationValid,
          },
      .raw_calibration =
          {
              .read = PositionSensorMt6816_RawRead,
              .prepare_clockwise = PositionSensorMt6816_RawPrepareClockwise,
              .set_direction_and_rebase =
                  PositionSensorMt6816_RawSetDirectionAndRebase,
              .commit_pole_pairs = PositionSensorMt6816_RawCommitPolePairs,
              .commit_offset_and_lut =
                  PositionSensorMt6816_RawCommitOffsetAndLut,
          },
  };
  return &adapter;
}
