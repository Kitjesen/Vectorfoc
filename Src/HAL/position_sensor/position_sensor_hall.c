#include "position_sensor_internal.h"

#include "board_config.h"

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL &&                      \
    (!defined(TEST_ENV) || defined(POSITION_SENSOR_TEST_USE_DRIVERS))
#include "hall_encoder.h"
#define POSITION_SENSOR_HALL_INIT_AVAILABLE 1
#else
#define POSITION_SENSOR_HALL_INIT_AVAILABLE 0
#endif

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL && !defined(TEST_ENV)
#include "motor_hal_api.h"

#include <string.h>

#define POSITION_SENSOR_HALL_RUNTIME_AVAILABLE 1
extern const Motor_HAL_EncoderInterface_t g_hall_encoder_interface;
#else
#define POSITION_SENSOR_HALL_RUNTIME_AVAILABLE 0
#endif

static PositionSensorStatus_t PositionSensorHall_Init(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
#if POSITION_SENSOR_HALL_INIT_AVAILABLE
  Hall_Init();
#endif
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorHall_UpdateAndRead(PositionSensorSample_t *sample) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  Motor_HAL_EncoderData_t data;
  if (!g_hall_encoder_interface.update()) {
    return POSITION_SENSOR_STATUS_IO_ERROR;
  }
  g_hall_encoder_interface.get_data(&data);
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
PositionSensorHall_SetPolePairs(uint8_t pole_pairs) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  g_hall_encoder_interface.set_pole_pairs(pole_pairs);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)pole_pairs;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorHall_ZeroMechanical(void) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  g_hall_encoder_interface.zero_position();
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorHall_SetElectricalOffset(float offset_rad) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  g_hall_encoder_interface.set_offset(offset_rad);
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorHall_GetElectricalOffset(float *offset_rad) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  *offset_rad = g_hall_encoder_interface.get_offset();
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t
PositionSensorHall_GetHealth(PositionSensorDriverHealth_t *health) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  health->valid = Hall_IsSignalValid();
  health->diagnostic_flags =
      health->valid ? 0u : POSITION_SENSOR_DIAGNOSTIC_SIGNAL_INVALID;
  health->transport_error_score = 0u;
  health->calibrated = hall_data.calib_valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)health;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorHall_CaptureCalibration(
    PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  snapshot->valid = hall_data.calib_valid;
  memset(snapshot->offset_lut, 0, sizeof(snapshot->offset_lut));
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorHall_RestoreCalibration(
    const PositionSensorCalibrationSnapshot_t *snapshot) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  hall_data.calib_valid = snapshot->valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)snapshot;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorHall_ClearCalibration(void) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  hall_data.calib_valid = false;
  g_hall_encoder_interface.set_offset(0.0f);
  return POSITION_SENSOR_STATUS_OK;
#else
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

static PositionSensorStatus_t PositionSensorHall_SetCalibrationValid(
    bool valid) {
#if POSITION_SENSOR_HALL_RUNTIME_AVAILABLE
  hall_data.calib_valid = valid;
  return POSITION_SENSOR_STATUS_OK;
#else
  (void)valid;
  return POSITION_SENSOR_STATUS_UNSUPPORTED;
#endif
}

const PositionSensorAdapter_t *PositionSensorHall_GetAdapter(void) {
  static const PositionSensorAdapter_t adapter = {
      .descriptor =
          {
              .name = "Hall",
              .capabilities = POSITION_SENSOR_CAP_COMMUTATION |
                              POSITION_SENSOR_CAP_HEALTH |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION,
          },
      .runtime =
          {
              .init = PositionSensorHall_Init,
              .update_and_read = PositionSensorHall_UpdateAndRead,
              .set_pole_pairs = PositionSensorHall_SetPolePairs,
              .zero_mechanical_position = PositionSensorHall_ZeroMechanical,
              .set_electrical_offset = PositionSensorHall_SetElectricalOffset,
              .get_electrical_offset = PositionSensorHall_GetElectricalOffset,
              .get_health = PositionSensorHall_GetHealth,
          },
      .calibration =
          {
              .capture = PositionSensorHall_CaptureCalibration,
              .restore = PositionSensorHall_RestoreCalibration,
              .clear = PositionSensorHall_ClearCalibration,
              .set_valid = PositionSensorHall_SetCalibrationValid,
          },
      /* Hall has neither a raw direction/pole workflow nor a 128-entry LUT. */
      .raw_calibration = {0},
  };
  return &adapter;
}
