#include "position_sensor.h"
#include "position_sensor_motor_hal.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static uint32_t s_set_pole_pairs_calls;
static uint8_t s_last_pole_pairs;
static PositionSensorSample_t s_sample;

PositionSensorStatus_t PositionSensor_UpdateAndRead(
    uint8_t pole_pairs, PositionSensorSample_t *sample) {
  s_last_pole_pairs = pole_pairs;
  if (sample != NULL) {
    *sample = s_sample;
  }
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_GetLastSample(
    PositionSensorSample_t *sample) {
  *sample = s_sample;
  return POSITION_SENSOR_STATUS_OK;
}

bool PositionSensor_IsInitialized(void) { return true; }

PositionSensorStatus_t PositionSensor_SetPolePairs(uint8_t pole_pairs) {
  s_set_pole_pairs_calls++;
  s_last_pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_ZeroMechanicalPosition(void) {
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_SetElectricalOffset(float offset_rad) {
  (void)offset_rad;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_GetElectricalOffset(float *offset_rad) {
  *offset_rad = 0.25f;
  return POSITION_SENSOR_STATUS_OK;
}

int main(void) {
  Motor_HAL_EncoderData_t data;

  s_sample.position_rad = 9.0f;
  s_sample.mechanical_angle_rad = 1.0f;
  s_sample.velocity_rad_s = -2.0f;
  s_sample.electrical_angle_rad = 3.0f;
  s_sample.native_raw = 123;

  /* One pole pair is a valid first configuration and must not be mistaken for
   * an already-applied default. Repeated ISR calls are then free. */
  g_position_sensor_motor_hal_interface.set_pole_pairs(1u);
  assert(s_set_pole_pairs_calls == 1u);
  g_position_sensor_motor_hal_interface.set_pole_pairs(1u);
  assert(s_set_pole_pairs_calls == 1u);
  assert(g_position_sensor_motor_hal_interface.update());
  assert(s_last_pole_pairs == 1u);

  g_position_sensor_motor_hal_interface.set_pole_pairs(7u);
  assert(s_set_pole_pairs_calls == 2u);
  assert(g_position_sensor_motor_hal_interface.update());
  assert(s_last_pole_pairs == 7u);

  g_position_sensor_motor_hal_interface.get_data(&data);
  assert(fabsf(data.position_rad - 9.0f) < 1e-6f);
  assert(fabsf(data.angle_rad - 1.0f) < 1e-6f);
  assert(fabsf(data.velocity_rad + 2.0f) < 1e-6f);
  assert(fabsf(data.elec_angle - 3.0f) < 1e-6f);
  assert(data.raw_value == 123);
  assert(fabsf(g_position_sensor_motor_hal_interface.get_offset() - 0.25f) <
         1e-6f);

  puts("Position sensor Motor HAL compatibility tests PASSED");
  return 0;
}
