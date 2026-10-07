#include "position_sensor.h"
#include "position_sensor_internal.h"

#include <assert.h>
#include <math.h>
#include <string.h>

typedef struct {
  unsigned init_calls;
  unsigned update_calls;
  unsigned set_pole_pairs_calls;
  uint8_t pole_pairs;
  float offset;
  PositionSensorStatus_t next_status;
  PositionSensorSample_t next_sample;
} FakeSensor;

static FakeSensor fake;

static PositionSensorStatus_t fake_init(void) {
  ++fake.init_calls;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
fake_update(PositionSensorSample_t *sample) {
  ++fake.update_calls;
  if (fake.next_status != POSITION_SENSOR_STATUS_OK) {
    PositionSensorStatus_t status = fake.next_status;
    fake.next_status = POSITION_SENSOR_STATUS_OK;
    return status;
  }
  *sample = fake.next_sample;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t fake_set_pole_pairs(uint8_t pole_pairs) {
  ++fake.set_pole_pairs_calls;
  fake.pole_pairs = pole_pairs;
  fake.offset = 0.0f;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t fake_set_offset(float offset) {
  fake.offset = offset;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t fake_get_offset(float *offset) {
  *offset = fake.offset;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t
fake_get_health(PositionSensorDriverHealth_t *health) {
  health->valid = true;
  health->diagnostic_flags = 0x5au;
  health->transport_error_score = 7u;
  health->calibrated = true;
  return POSITION_SENSOR_STATUS_OK;
}

const PositionSensorAdapter_t *PositionSensor_SelectAdapter(void) {
  static const PositionSensorAdapter_t adapter = {
      .descriptor = {.name = "fake",
                     .capabilities = POSITION_SENSOR_CAP_ABSOLUTE |
                                     POSITION_SENSOR_CAP_HEALTH},
      .runtime = {.init = fake_init,
                  .update_and_read = fake_update,
                  .set_pole_pairs = fake_set_pole_pairs,
                  .set_electrical_offset = fake_set_offset,
                  .get_electrical_offset = fake_get_offset,
                  .get_health = fake_get_health},
  };
  return &adapter;
}

static void initializes_once_and_requires_a_sample_before_cached_read(void) {
  PositionSensorSample_t sample;

  assert(PositionSensor_GetLastSample(&sample) ==
         POSITION_SENSOR_STATUS_NOT_INITIALIZED);
  assert(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);
  assert(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);
  assert(fake.init_calls == 1u);
  assert(PositionSensor_GetLastSample(&sample) == POSITION_SENSOR_STATUS_NOT_READY);
}

static void pole_pair_change_preserves_electrical_offset(void) {
  PositionSensorSample_t sample;
  fake.offset = 1.25f;
  fake.next_sample = (PositionSensorSample_t){.position_rad = 12.5f,
                                               .mechanical_angle_rad = 0.5f,
                                               .velocity_rad_s = 3.0f,
                                               .electrical_angle_rad = -1.0f,
                                               .native_raw = 123};

  assert(PositionSensor_UpdateAndRead(7u, &sample) == POSITION_SENSOR_STATUS_OK);
  assert(fake.set_pole_pairs_calls == 1u);
  assert(fake.pole_pairs == 7u);
  assert(fake.offset == 1.25f);
  assert(memcmp(&sample, &fake.next_sample, sizeof(sample)) == 0);
}

static void invalid_frame_is_rejected_without_replacing_cached_sample(void) {
  PositionSensorSample_t cached;
  PositionSensorHealth_t health;
  const PositionSensorSample_t good = fake.next_sample;

  fake.next_sample.position_rad = NAN;
  assert(PositionSensor_UpdateAndRead(7u, &cached) == POSITION_SENSOR_STATUS_IO_ERROR);
  assert(PositionSensor_GetLastSample(&cached) == POSITION_SENSOR_STATUS_OK);
  assert(memcmp(&cached, &good, sizeof(cached)) == 0);
  assert(PositionSensor_GetHealth(&health) == POSITION_SENSOR_STATUS_OK);
  assert(!health.valid);
  assert(health.consecutive_failures == 1u);
  assert(health.total_failures == 1u);

  fake.next_sample = good;
  assert(PositionSensor_UpdateAndRead(7u, &cached) == POSITION_SENSOR_STATUS_OK);
  assert(PositionSensor_GetHealth(&health) == POSITION_SENSOR_STATUS_OK);
  assert(health.valid);
  assert(health.consecutive_failures == 0u);
  assert(health.total_failures == 1u);
}

int main(void) {
  initializes_once_and_requires_a_sample_before_cached_read();
  pole_pair_change_preserves_electrical_offset();
  invalid_frame_is_rejected_without_replacing_cached_sample();
  return 0;
}
