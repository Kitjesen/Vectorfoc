#include "position_sensor.h"

#include "platform.h"
#include "position_sensor_internal.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized;
static bool s_has_last_sample;
static uint8_t s_pole_pairs;
static uint32_t s_consecutive_failures;
static uint32_t s_total_failures;
static PositionSensorSample_t s_last_sample;

static const PositionSensorAdapter_t *PositionSensor_GetAdapter(void) {
  return PositionSensor_SelectAdapter();
}

static bool
PositionSensor_HasCapability(const PositionSensorAdapter_t *adapter,
                             PositionSensorCapabilities_t capability) {
  return adapter != NULL &&
         (adapter->descriptor.capabilities & capability) == capability;
}

static void PositionSensor_RecordFailure(void) {
  CRITICAL_SECTION_BEGIN();
  if (s_consecutive_failures < UINT32_MAX) {
    s_consecutive_failures++;
  }
  if (s_total_failures < UINT32_MAX) {
    s_total_failures++;
  }
  CRITICAL_SECTION_END();
}

static PositionSensorStatus_t
PositionSensor_RequireInitialized(const PositionSensorAdapter_t **adapter) {
  if (!s_initialized) {
    return POSITION_SENSOR_STATUS_NOT_INITIALIZED;
  }
  *adapter = PositionSensor_GetAdapter();
  return *adapter != NULL ? POSITION_SENSOR_STATUS_OK
                          : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

const PositionSensorDescriptor_t *PositionSensor_GetDescriptor(void) {
  const PositionSensorAdapter_t *adapter = PositionSensor_GetAdapter();
  return adapter != NULL ? &adapter->descriptor : NULL;
}

PositionSensorStatus_t PositionSensor_Init(void) {
  const PositionSensorAdapter_t *adapter = PositionSensor_GetAdapter();
  PositionSensorStatus_t status;

  if (adapter == NULL || adapter->runtime.init == NULL) {
    s_initialized = false;
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  if (s_initialized) {
    return POSITION_SENSOR_STATUS_OK;
  }

  status = adapter->runtime.init();
  if (status != POSITION_SENSOR_STATUS_OK) {
    s_initialized = false;
    return status;
  }

  s_initialized = true;
  s_has_last_sample = false;
  s_pole_pairs = 0u;
  return POSITION_SENSOR_STATUS_OK;
}

bool PositionSensor_IsInitialized(void) { return s_initialized; }

static PositionSensorStatus_t PositionSensor_SetPolePairsWithAdapter(
    const PositionSensorAdapter_t *adapter, uint8_t pole_pairs) {
  PositionSensorStatus_t status;
  float preserved_offset;

  if (s_pole_pairs == pole_pairs) {
    return POSITION_SENSOR_STATUS_OK;
  }
  if (adapter->runtime.set_pole_pairs == NULL ||
      adapter->runtime.get_electrical_offset == NULL ||
      adapter->runtime.set_electrical_offset == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }

  status = adapter->runtime.get_electrical_offset(&preserved_offset);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  status = adapter->runtime.set_pole_pairs(pole_pairs);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  status = adapter->runtime.set_electrical_offset(preserved_offset);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }

  s_pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_SetPolePairs(uint8_t pole_pairs) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (pole_pairs == 0u) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  return status == POSITION_SENSOR_STATUS_OK
             ? PositionSensor_SetPolePairsWithAdapter(adapter, pole_pairs)
             : status;
}

PositionSensorStatus_t
PositionSensor_UpdateAndRead(uint8_t pole_pairs,
                             PositionSensorSample_t *sample) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  PositionSensorSample_t next_sample;

  if (pole_pairs == 0u || sample == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }

  status = PositionSensor_SetPolePairsWithAdapter(adapter, pole_pairs);
  if (status != POSITION_SENSOR_STATUS_OK) {
    PositionSensor_RecordFailure();
    return status;
  }
  if (adapter->runtime.update_and_read == NULL) {
    PositionSensor_RecordFailure();
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }

  status = adapter->runtime.update_and_read(&next_sample);
  if (status != POSITION_SENSOR_STATUS_OK ||
      !isfinite(next_sample.position_rad) ||
      !isfinite(next_sample.mechanical_angle_rad) ||
      !isfinite(next_sample.velocity_rad_s) ||
      !isfinite(next_sample.electrical_angle_rad)) {
    PositionSensor_RecordFailure();
    return status == POSITION_SENSOR_STATUS_OK ? POSITION_SENSOR_STATUS_IO_ERROR
                                               : status;
  }

  /* The writer is normally the 20 kHz ISR, while task/UI readers may copy the
   * multi-field frame.  A short interrupt-masked publication prevents a
   * higher-priority reader from observing a partially assigned structure. */
  CRITICAL_SECTION_BEGIN();
  s_consecutive_failures = 0u;
  s_last_sample = next_sample;
  s_has_last_sample = true;
  CRITICAL_SECTION_END();
  *sample = next_sample;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t
PositionSensor_GetLastSample(PositionSensorSample_t *sample) {
  PositionSensorStatus_t status;
  if (sample == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }

  CRITICAL_SECTION_BEGIN();
  if (!s_initialized) {
    status = POSITION_SENSOR_STATUS_NOT_INITIALIZED;
  } else if (!s_has_last_sample) {
    status = POSITION_SENSOR_STATUS_NOT_READY;
  } else {
    *sample = s_last_sample;
    status = POSITION_SENSOR_STATUS_OK;
  }
  CRITICAL_SECTION_END();
  return status;
}

PositionSensorStatus_t PositionSensor_ZeroMechanicalPosition(void) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return adapter->runtime.zero_mechanical_position != NULL
             ? adapter->runtime.zero_mechanical_position()
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t PositionSensor_SetElectricalOffset(float offset_rad) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (!isfinite(offset_rad)) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return adapter->runtime.set_electrical_offset != NULL
             ? adapter->runtime.set_electrical_offset(offset_rad)
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t PositionSensor_GetElectricalOffset(float *offset_rad) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (offset_rad == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return adapter->runtime.get_electrical_offset != NULL
             ? adapter->runtime.get_electrical_offset(offset_rad)
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t
PositionSensor_GetHealth(PositionSensorHealth_t *health) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorDriverHealth_t driver_health;
  PositionSensorStatus_t status;
  uint32_t consecutive_failures;
  uint32_t total_failures;

  if (health == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (adapter->runtime.get_health == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }

  memset(&driver_health, 0, sizeof(driver_health));
  status = adapter->runtime.get_health(&driver_health);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }

  CRITICAL_SECTION_BEGIN();
  consecutive_failures = s_consecutive_failures;
  total_failures = s_total_failures;
  CRITICAL_SECTION_END();

  health->valid = driver_health.valid && consecutive_failures == 0u;
  health->consecutive_failures = consecutive_failures;
  health->total_failures = total_failures;
  health->diagnostic_flags = driver_health.diagnostic_flags;
  health->transport_error_score = driver_health.transport_error_score;
  health->calibrated = driver_health.calibrated;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_CaptureCalibration(
    PositionSensorCalibrationSnapshot_t *snapshot) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (snapshot == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (!PositionSensor_HasCapability(
          adapter, POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION) ||
      adapter->calibration.capture == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  return adapter->calibration.capture(snapshot);
}

PositionSensorStatus_t PositionSensor_RestoreCalibration(
    const PositionSensorCalibrationSnapshot_t *snapshot) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (snapshot == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (!PositionSensor_HasCapability(
          adapter, POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION) ||
      adapter->calibration.restore == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  return adapter->calibration.restore(snapshot);
}

PositionSensorStatus_t PositionSensor_ClearCalibration(void) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (!PositionSensor_HasCapability(
          adapter, POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION) ||
      adapter->calibration.clear == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  return adapter->calibration.clear();
}

PositionSensorStatus_t PositionSensor_SetCalibrationValid(bool valid) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status = PositionSensor_RequireInitialized(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (!PositionSensor_HasCapability(
          adapter, POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION) ||
      adapter->calibration.set_valid == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  return adapter->calibration.set_valid(valid);
}

static PositionSensorStatus_t
PositionSensor_RequireRawCalibration(const PositionSensorAdapter_t **adapter) {
  PositionSensorStatus_t status = PositionSensor_RequireInitialized(adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return PositionSensor_HasCapability(*adapter,
                                      POSITION_SENSOR_CAP_RAW_DIRECTION_POLE)
             ? POSITION_SENSOR_STATUS_OK
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t
PositionSensor_RawCalibrationRead(PositionSensorRawCalibrationState_t *state) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (state == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireRawCalibration(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return adapter->raw_calibration.read != NULL
             ? adapter->raw_calibration.read(state)
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t PositionSensor_RawCalibrationPrepareClockwise(void) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status =
      PositionSensor_RequireRawCalibration(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return adapter->raw_calibration.prepare_clockwise != NULL
             ? adapter->raw_calibration.prepare_clockwise()
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t PositionSensor_RawCalibrationSetDirectionAndRebase(
    PositionSensorDirection_t direction) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (direction != POSITION_SENSOR_DIRECTION_CLOCKWISE &&
      direction != POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireRawCalibration(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  return adapter->raw_calibration.set_direction_and_rebase != NULL
             ? adapter->raw_calibration.set_direction_and_rebase(direction)
             : POSITION_SENSOR_STATUS_UNSUPPORTED;
}

PositionSensorStatus_t
PositionSensor_RawCalibrationCommitPolePairs(uint8_t pole_pairs) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (pole_pairs == 0u) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireRawCalibration(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (adapter->raw_calibration.commit_pole_pairs == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  status = adapter->raw_calibration.commit_pole_pairs(pole_pairs);
  if (status == POSITION_SENSOR_STATUS_OK) {
    s_pole_pairs = pole_pairs;
  }
  return status;
}

PositionSensorStatus_t PositionSensor_RawCalibrationCommitOffsetAndLut(
    int32_t offset_counts,
    const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]) {
  const PositionSensorAdapter_t *adapter;
  PositionSensorStatus_t status;
  if (offset_lut == NULL) {
    return POSITION_SENSOR_STATUS_INVALID_ARGUMENT;
  }
  status = PositionSensor_RequireRawCalibration(&adapter);
  if (status != POSITION_SENSOR_STATUS_OK) {
    return status;
  }
  if (!PositionSensor_HasCapability(
          adapter, POSITION_SENSOR_CAP_LINEARITY_CALIBRATION) ||
      adapter->raw_calibration.commit_offset_and_lut == NULL) {
    return POSITION_SENSOR_STATUS_UNSUPPORTED;
  }
  return adapter->raw_calibration.commit_offset_and_lut(offset_counts,
                                                        offset_lut);
}
