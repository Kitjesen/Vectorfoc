#include "position_sensor.h"
#include "position_sensor_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);              \
      return 1;                                                                \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                \
  CHECK(fabsf((actual) - (expected)) <= (tolerance))

typedef struct {
  uint32_t init_calls;
  uint32_t update_calls;
  uint32_t set_pole_pairs_calls;
  uint32_t zero_calls;
  uint32_t restore_calls;
  uint32_t clear_calls;
  uint8_t pole_pairs;
  float electrical_offset_rad;
  bool calibrated;
  int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE];
  PositionSensorStatus_t next_update_status;
  PositionSensorSample_t sample;
  PositionSensorRawCalibrationState_t raw;
} FakeSensorState_t;

static FakeSensorState_t s_fake;

static PositionSensorStatus_t Fake_Init(void) {
  s_fake.init_calls++;
  /* Model the real TMR3109 behavior: init clears persisted calibration. */
  s_fake.calibrated = false;
  memset(s_fake.offset_lut, 0, sizeof(s_fake.offset_lut));
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
Fake_UpdateAndRead(PositionSensorSample_t *sample) {
  s_fake.update_calls++;
  if (s_fake.next_update_status != POSITION_SENSOR_STATUS_OK) {
    PositionSensorStatus_t status = s_fake.next_update_status;
    s_fake.next_update_status = POSITION_SENSOR_STATUS_OK;
    return status;
  }
  *sample = s_fake.sample;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_SetPolePairs(uint8_t pole_pairs) {
  s_fake.set_pole_pairs_calls++;
  s_fake.pole_pairs = pole_pairs;
  /* Deliberately destructive: the module must restore the electrical offset. */
  s_fake.electrical_offset_rad = 0.0f;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_ZeroMechanicalPosition(void) {
  s_fake.zero_calls++;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_SetElectricalOffset(float offset_rad) {
  s_fake.electrical_offset_rad = offset_rad;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_GetElectricalOffset(float *offset_rad) {
  *offset_rad = s_fake.electrical_offset_rad;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
Fake_GetHealth(PositionSensorDriverHealth_t *health) {
  health->valid = true;
  health->diagnostic_flags = 0x5Au;
  health->transport_error_score = 17u;
  health->calibrated = s_fake.calibrated;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
Fake_CaptureCalibration(PositionSensorCalibrationSnapshot_t *snapshot) {
  snapshot->valid = s_fake.calibrated;
#if defined(POSITION_SENSOR_TEST_PERSISTENT_ONLY)
  memset(snapshot->offset_lut, 0, sizeof(snapshot->offset_lut));
#else
  memcpy(snapshot->offset_lut, s_fake.offset_lut, sizeof(snapshot->offset_lut));
#endif
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
Fake_RestoreCalibration(const PositionSensorCalibrationSnapshot_t *snapshot) {
  s_fake.restore_calls++;
  s_fake.calibrated = snapshot->valid;
#if defined(POSITION_SENSOR_TEST_PERSISTENT_ONLY)
  memset(s_fake.offset_lut, 0, sizeof(s_fake.offset_lut));
#else
  memcpy(s_fake.offset_lut, snapshot->offset_lut, sizeof(s_fake.offset_lut));
#endif
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_ClearCalibration(void) {
  s_fake.clear_calls++;
  s_fake.calibrated = false;
  s_fake.electrical_offset_rad = 0.0f;
  memset(s_fake.offset_lut, 0, sizeof(s_fake.offset_lut));
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_SetCalibrationValid(bool valid) {
  s_fake.calibrated = valid;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
Fake_RawRead(PositionSensorRawCalibrationState_t *state) {
  *state = s_fake.raw;
  state->pole_pairs = s_fake.pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_RawPrepareClockwise(void) {
  s_fake.raw.direction = POSITION_SENSOR_DIRECTION_CLOCKWISE;
  s_fake.raw.shadow_count = 0;
  s_fake.raw.count_in_cpr = 0;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
Fake_RawSetDirectionAndRebase(PositionSensorDirection_t direction) {
  s_fake.raw.direction = direction;
  s_fake.raw.shadow_count = 0;
  s_fake.raw.count_in_cpr = 0;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_RawCommitPolePairs(uint8_t pole_pairs) {
  s_fake.pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_RawCommitOffsetAndLut(
    int32_t offset_counts,
    const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]) {
  s_fake.raw.offset_counts = offset_counts;
  memcpy(s_fake.offset_lut, offset_lut, sizeof(s_fake.offset_lut));
  return POSITION_SENSOR_STATUS_OK;
}

const PositionSensorAdapter_t *PositionSensor_SelectAdapter(void) {
  static const PositionSensorAdapter_t adapter = {
      .descriptor =
          {
              .name = "fake",
#if defined(POSITION_SENSOR_TEST_PERSISTENT_ONLY)
              .capabilities = POSITION_SENSOR_CAP_HEALTH |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION,
#else
              .capabilities = POSITION_SENSOR_CAP_ABSOLUTE |
                              POSITION_SENSOR_CAP_HEALTH |
                              POSITION_SENSOR_CAP_RAW_DIRECTION_POLE |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION |
                              POSITION_SENSOR_CAP_LINEARITY_CALIBRATION,
#endif
          },
      .runtime =
          {
              .init = Fake_Init,
              .update_and_read = Fake_UpdateAndRead,
              .set_pole_pairs = Fake_SetPolePairs,
              .zero_mechanical_position = Fake_ZeroMechanicalPosition,
              .set_electrical_offset = Fake_SetElectricalOffset,
              .get_electrical_offset = Fake_GetElectricalOffset,
              .get_health = Fake_GetHealth,
          },
      .calibration =
          {
              .capture = Fake_CaptureCalibration,
              .restore = Fake_RestoreCalibration,
              .clear = Fake_ClearCalibration,
              .set_valid = Fake_SetCalibrationValid,
          },
      .raw_calibration =
          {
              .read = Fake_RawRead,
              .prepare_clockwise = Fake_RawPrepareClockwise,
              .set_direction_and_rebase = Fake_RawSetDirectionAndRebase,
              .commit_pole_pairs = Fake_RawCommitPolePairs,
              .commit_offset_and_lut = Fake_RawCommitOffsetAndLut,
          },
  };
  return &adapter;
}

static void FillSnapshot(PositionSensorCalibrationSnapshot_t *snapshot) {
  snapshot->valid = true;
  for (uint32_t i = 0; i < POSITION_SENSOR_CALIBRATION_LUT_SIZE; ++i) {
    snapshot->offset_lut[i] = (int16_t)((int32_t)i - 64);
  }
}

#if defined(POSITION_SENSOR_TEST_PERSISTENT_ONLY)
int main(void) {
  PositionSensorCalibrationSnapshot_t requested;
  PositionSensorCalibrationSnapshot_t captured;
  PositionSensorRawCalibrationState_t raw;

  memset(&s_fake, 0, sizeof(s_fake));
  s_fake.calibrated = true;
  s_fake.electrical_offset_rad = 2.5f;
  FillSnapshot(&requested);

  CHECK((PositionSensor_GetDescriptor()->capabilities &
         POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION) != 0u);
  CHECK((PositionSensor_GetDescriptor()->capabilities &
         POSITION_SENSOR_CAP_LINEARITY_CALIBRATION) == 0u);
  CHECK((PositionSensor_GetDescriptor()->capabilities &
         POSITION_SENSOR_CAP_RAW_DIRECTION_POLE) == 0u);

  /* Persistence is intentionally ordered after driver init to avoid a second
   * full LUT staging buffer in scarce MCU SRAM. */
  CHECK(PositionSensor_RestoreCalibration(&requested) ==
        POSITION_SENSOR_STATUS_NOT_INITIALIZED);
  CHECK(PositionSensor_ClearCalibration() ==
        POSITION_SENSOR_STATUS_NOT_INITIALIZED);
  CHECK(s_fake.clear_calls == 0u);
  CHECK(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.init_calls == 1u);
  CHECK(PositionSensor_RestoreCalibration(&requested) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.restore_calls == 1u);
  CHECK(s_fake.calibrated);
  CHECK(PositionSensor_ClearCalibration() == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.restore_calls == 1u);
  CHECK(s_fake.clear_calls == 1u);
  CHECK(!s_fake.calibrated);
  CHECK_NEAR(s_fake.electrical_offset_rad, 0.0f, 1e-6f);

  CHECK(PositionSensor_CaptureCalibration(&captured) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(!captured.valid);
  for (uint32_t i = 0; i < POSITION_SENSOR_CALIBRATION_LUT_SIZE; ++i) {
    CHECK(captured.offset_lut[i] == 0);
  }

  CHECK(PositionSensor_SetCalibrationValid(true) == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.calibrated);
  CHECK(PositionSensor_SetCalibrationValid(false) == POSITION_SENSOR_STATUS_OK);
  CHECK(!s_fake.calibrated);

  CHECK(PositionSensor_RawCalibrationRead(&raw) ==
        POSITION_SENSOR_STATUS_UNSUPPORTED);
  CHECK(PositionSensor_RawCalibrationPrepareClockwise() ==
        POSITION_SENSOR_STATUS_UNSUPPORTED);
  CHECK(PositionSensor_RawCalibrationCommitOffsetAndLut(
            1, requested.offset_lut) == POSITION_SENSOR_STATUS_UNSUPPORTED);

  puts("Position sensor persistent-only tests PASSED");
  return 0;
}
#else
int main(void) {
  PositionSensorCalibrationSnapshot_t expected_calibration;
  PositionSensorCalibrationSnapshot_t captured_calibration;
  PositionSensorSample_t sample;
  PositionSensorSample_t last_sample;
  PositionSensorHealth_t health;
  PositionSensorRawCalibrationState_t raw;
  float electrical_offset_rad = 0.0f;

  memset(&s_fake, 0, sizeof(s_fake));
  s_fake.pole_pairs = 7u;
  s_fake.raw.cpr = 16384u;
  s_fake.raw.shadow_count = 1234567890123LL;
  s_fake.raw.count_in_cpr = 42;
  s_fake.raw.direction = POSITION_SENSOR_DIRECTION_CLOCKWISE;
  s_fake.sample.position_rad = 12.5f;
  s_fake.sample.mechanical_angle_rad = 0.75f;
  s_fake.sample.velocity_rad_s = -3.5f;
  s_fake.sample.electrical_angle_rad = 2.25f;
  s_fake.sample.native_raw = 1234;

  CHECK(PositionSensor_GetLastSample(&last_sample) ==
        POSITION_SENSOR_STATUS_NOT_INITIALIZED);
  CHECK(PositionSensor_UpdateAndRead(7u, &sample) ==
        POSITION_SENSOR_STATUS_NOT_INITIALIZED);

  /* Driver init clears its LUT, so persistence restore is deliberately next. */
  FillSnapshot(&expected_calibration);
  CHECK(PositionSensor_RestoreCalibration(&expected_calibration) ==
        POSITION_SENSOR_STATUS_NOT_INITIALIZED);
  CHECK(s_fake.restore_calls == 0u);
  CHECK(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.init_calls == 1u);
  CHECK(PositionSensor_RestoreCalibration(&expected_calibration) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.restore_calls == 1u);
  CHECK(s_fake.calibrated);
  CHECK(memcmp(s_fake.offset_lut, expected_calibration.offset_lut,
               sizeof(s_fake.offset_lut)) == 0);

  CHECK(PositionSensor_CaptureCalibration(&captured_calibration) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(captured_calibration.valid);
  CHECK(memcmp(captured_calibration.offset_lut, expected_calibration.offset_lut,
               sizeof(captured_calibration.offset_lut)) == 0);

  CHECK(PositionSensor_SetElectricalOffset(1.25f) == POSITION_SENSOR_STATUS_OK);
  CHECK(PositionSensor_SetPolePairs(7u) == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.set_pole_pairs_calls == 1u);
  CHECK(PositionSensor_GetElectricalOffset(&electrical_offset_rad) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK_NEAR(electrical_offset_rad, 1.25f, 1e-6f);

  CHECK(PositionSensor_UpdateAndRead(9u, &sample) == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.pole_pairs == 9u);
  CHECK(s_fake.set_pole_pairs_calls == 2u);
  CHECK_NEAR(s_fake.electrical_offset_rad, 1.25f, 1e-6f);
  CHECK_NEAR(sample.position_rad, 12.5f, 1e-6f);
  CHECK_NEAR(sample.mechanical_angle_rad, 0.75f, 1e-6f);
  CHECK_NEAR(sample.velocity_rad_s, -3.5f, 1e-6f);
  CHECK_NEAR(sample.electrical_angle_rad, 2.25f, 1e-6f);
  CHECK(sample.native_raw == 1234);

  CHECK(PositionSensor_GetLastSample(&last_sample) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(memcmp(&last_sample, &sample, sizeof(sample)) == 0);

  CHECK(PositionSensor_GetHealth(&health) == POSITION_SENSOR_STATUS_OK);
  CHECK(health.valid);
  CHECK(health.consecutive_failures == 0u);
  CHECK(health.total_failures == 0u);
  CHECK(health.diagnostic_flags == 0x5Au);
  CHECK(health.transport_error_score == 17u);
  CHECK(health.calibrated);

  s_fake.next_update_status = POSITION_SENSOR_STATUS_IO_ERROR;
  CHECK(PositionSensor_UpdateAndRead(9u, &sample) ==
        POSITION_SENSOR_STATUS_IO_ERROR);
  s_fake.next_update_status = POSITION_SENSOR_STATUS_IO_ERROR;
  CHECK(PositionSensor_UpdateAndRead(9u, &sample) ==
        POSITION_SENSOR_STATUS_IO_ERROR);
  CHECK(PositionSensor_GetHealth(&health) == POSITION_SENSOR_STATUS_OK);
  CHECK(!health.valid);
  CHECK(health.consecutive_failures == 2u);
  CHECK(health.total_failures == 2u);

  s_fake.sample.native_raw = 5678;
  CHECK(PositionSensor_UpdateAndRead(9u, &sample) == POSITION_SENSOR_STATUS_OK);
  CHECK(PositionSensor_GetHealth(&health) == POSITION_SENSOR_STATUS_OK);
  CHECK(health.valid);
  CHECK(health.consecutive_failures == 0u);
  CHECK(health.total_failures == 2u);
  CHECK(PositionSensor_GetLastSample(&last_sample) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(last_sample.native_raw == 5678);

  CHECK(PositionSensor_ZeroMechanicalPosition() == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.zero_calls == 1u);

  CHECK(PositionSensor_RawCalibrationRead(&raw) == POSITION_SENSOR_STATUS_OK);
  CHECK(raw.shadow_count == 1234567890123LL);
  CHECK(raw.count_in_cpr == 42);
  CHECK(raw.cpr == 16384u);
  CHECK(raw.pole_pairs == 9u);
  CHECK(PositionSensor_RawCalibrationPrepareClockwise() ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(PositionSensor_RawCalibrationSetDirectionAndRebase(
            POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.raw.direction == POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE);
  CHECK(PositionSensor_RawCalibrationCommitPolePairs(11u) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(PositionSensor_RawCalibrationCommitOffsetAndLut(
            -321, expected_calibration.offset_lut) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.raw.offset_counts == -321);
  CHECK(PositionSensor_SetCalibrationValid(false) == POSITION_SENSOR_STATUS_OK);
  CHECK(!s_fake.calibrated);
  CHECK(PositionSensor_SetCalibrationValid(true) == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.calibrated);

  CHECK(PositionSensor_ClearCalibration() == POSITION_SENSOR_STATUS_OK);
  CHECK(s_fake.clear_calls == 1u);
  CHECK(PositionSensor_CaptureCalibration(&captured_calibration) ==
        POSITION_SENSOR_STATUS_OK);
  CHECK(!captured_calibration.valid);
  for (uint32_t i = 0; i < POSITION_SENSOR_CALIBRATION_LUT_SIZE; ++i) {
    CHECK(captured_calibration.offset_lut[i] == 0);
  }

  CHECK(PositionSensor_UpdateAndRead(0u, &sample) ==
        POSITION_SENSOR_STATUS_INVALID_ARGUMENT);
  CHECK(PositionSensor_UpdateAndRead(7u, NULL) ==
        POSITION_SENSOR_STATUS_INVALID_ARGUMENT);
  CHECK(PositionSensor_GetLastSample(NULL) ==
        POSITION_SENSOR_STATUS_INVALID_ARGUMENT);
  CHECK(PositionSensor_GetHealth(NULL) ==
        POSITION_SENSOR_STATUS_INVALID_ARGUMENT);
  CHECK(PositionSensor_SetElectricalOffset(NAN) ==
        POSITION_SENSOR_STATUS_INVALID_ARGUMENT);
  CHECK(PositionSensor_RawCalibrationSetDirectionAndRebase(
            (PositionSensorDirection_t)0) ==
        POSITION_SENSOR_STATUS_INVALID_ARGUMENT);

  puts("Position sensor runtime/maintenance tests PASSED");
  return 0;
}
#endif
