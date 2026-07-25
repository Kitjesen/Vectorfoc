#include "position_sensor_internal.h"

#include "board_config.h"

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ &&                       \
    (!defined(TEST_ENV) || defined(POSITION_SENSOR_TEST_USE_DRIVERS))
#include "abz_encoder.h"
#define POSITION_SENSOR_ABZ_INIT_AVAILABLE 1
#else
#define POSITION_SENSOR_ABZ_INIT_AVAILABLE 0
#endif

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ && !defined(TEST_ENV)
#include "motor_hal_api.h"

#include <math.h>
#include <string.h>

#define POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE 1
#else
#define POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE 0
#endif

static PositionSensorStatus_t PositionSensorAbz_Init(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
#if POSITION_SENSOR_ABZ_INIT_AVAILABLE
  Abz_Init();
#endif
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorAbz_UpdateAndRead(PositionSensorSample_t *sample) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  Motor_HAL_EncoderData_t data;
  if (!g_abz_encoder_interface.update()) {
    return POSITION_SENSOR_STATUS_IO_ERROR;
  }
  g_abz_encoder_interface.get_data(&data);
  sample->position_rad = data.position_rad;
  sample->mechanical_angle_rad = data.angle_rad;
  sample->velocity_rad_s = data.velocity_rad;
  sample->electrical_angle_rad = data.elec_angle;
  sample->native_raw = data.raw_value;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)sample;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorAbz_SetPolePairs(uint8_t pole_pairs) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  g_abz_encoder_interface.set_pole_pairs(pole_pairs);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)pole_pairs;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorAbz_ZeroMechanical(void) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  g_abz_encoder_interface.zero_position();
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorAbz_SetElectricalOffset(float offset_rad) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  g_abz_encoder_interface.set_offset(offset_rad);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorAbz_GetElectricalOffset(float *offset_rad) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  *offset_rad = g_abz_encoder_interface.get_offset();
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorAbz_GetHealth(PositionSensorDriverHealth_t *health) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  health->valid = abz_data.cpr > 0u && abz_data.pole_pairs > 0u &&
                  isfinite(abz_data.mec_angle_rad) &&
                  isfinite(abz_data.elec_angle_rad) &&
                  isfinite(abz_data.velocity_rad_s);
  health->diagnostic_flags =
      health->valid ? 0u : POSITION_SENSOR_DIAGNOSTIC_SIGNAL_INVALID;
  health->transport_error_score = 0u;
  health->calibrated = abz_data.calib_valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)health;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorAbz_CaptureCalibration(
    PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  snapshot->valid = abz_data.calib_valid;
  memset(snapshot->offset_lut, 0, sizeof(snapshot->offset_lut));
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorAbz_RestoreCalibration(
    const PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  abz_data.calib_valid = snapshot->valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorAbz_ClearCalibration(void) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  abz_data.calib_valid = false;
  g_abz_encoder_interface.set_offset(0.0f);
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorAbz_SetCalibrationValid(bool valid) {
#if POSITION_SENSOR_ABZ_RUNTIME_AVAILABLE
  abz_data.calib_valid = valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)valid;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

const PositionSensorAdapter_t *PositionSensorAbz_GetAdapter(void) {
  static const PositionSensorAdapter_t adapter = {
      .descriptor =
          {
              .name = "ABZ",
              .capabilities = POSITION_SENSOR_CAP_INCREMENTAL |
                              POSITION_SENSOR_CAP_INDEX |
                              POSITION_SENSOR_CAP_HEALTH |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION,
          },
      .runtime =
          {
              .init = PositionSensorAbz_Init,
              .update_and_read = PositionSensorAbz_UpdateAndRead,
              .set_pole_pairs = PositionSensorAbz_SetPolePairs,
              .zero_mechanical_position = PositionSensorAbz_ZeroMechanical,
              .set_electrical_offset = PositionSensorAbz_SetElectricalOffset,
              .get_electrical_offset = PositionSensorAbz_GetElectricalOffset,
              .get_health = PositionSensorAbz_GetHealth,
          },
      .calibration =
          {
              .capture = PositionSensorAbz_CaptureCalibration,
              .restore = PositionSensorAbz_RestoreCalibration,
              .clear = PositionSensorAbz_ClearCalibration,
              .set_valid = PositionSensorAbz_SetCalibrationValid,
          },
      /* ABZ is intentionally not presented as a magnetic raw/LUT adapter. */
      .raw_calibration = {0},
  };
  return &adapter;
}
