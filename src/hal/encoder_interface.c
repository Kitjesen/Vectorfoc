#include "encoder_interface.h"

#include "motor_runtime.h"
#include <string.h>

static PositionSensorStatus_t s_last_update_status = POSITION_SENSOR_STATUS_NOT_INITIALIZED;

int MHAL_Encoder_Init(void) {
  s_last_update_status = PositionSensor_Init();
  return s_last_update_status == POSITION_SENSOR_STATUS_OK ? 0 : -1;
}

int MHAL_Encoder_Update(void) {
  PositionSensorSample_t sample;
  uint8_t pole_pairs = motor_data.parameters.pole_pairs > 0
                           ? (uint8_t)motor_data.parameters.pole_pairs
                           : 1u;
  s_last_update_status = PositionSensor_UpdateAndRead(pole_pairs, &sample);
  return s_last_update_status == POSITION_SENSOR_STATUS_OK ? 0 : -1;
}

PositionSensorStatus_t MHAL_Encoder_GetLastUpdateStatus(void) {
  return s_last_update_status;
}

void MHAL_Encoder_GetData(EncoderData *data) {
  PositionSensorSample_t sample;
  if (data == NULL) return;
  if (PositionSensor_GetLastSample(&sample) != POSITION_SENSOR_STATUS_OK) {
    memset(data, 0, sizeof(*data));
    return;
  }
  data->position_rad = sample.position_rad;
  data->angle_rad = sample.mechanical_angle_rad;
  data->velocity_rad = sample.velocity_rad_s;
  data->elec_angle = sample.electrical_angle_rad;
  data->raw_value = sample.native_raw;
}

float MHAL_Encoder_GetPosition(void) {
  PositionSensorSample_t sample;
  return PositionSensor_GetLastSample(&sample) == POSITION_SENSOR_STATUS_OK ? sample.position_rad : 0.0f;
}

float MHAL_Encoder_GetVelocity(void) {
  PositionSensorSample_t sample;
  return PositionSensor_GetLastSample(&sample) == POSITION_SENSOR_STATUS_OK ? sample.velocity_rad_s : 0.0f;
}

int MHAL_Encoder_SetOffset(float offset) {
  return PositionSensor_SetElectricalOffset(offset) == POSITION_SENSOR_STATUS_OK ? 0 : -1;
}

float MHAL_Encoder_GetOffset(void) {
  float offset = 0.0f;
  (void)PositionSensor_GetElectricalOffset(&offset);
  return offset;
}

int MHAL_Encoder_ZeroPosition(void) {
  return PositionSensor_ZeroMechanicalPosition() == POSITION_SENSOR_STATUS_OK ? 0 : -1;
}
