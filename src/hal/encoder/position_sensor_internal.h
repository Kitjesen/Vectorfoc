#ifndef POSITION_SENSOR_INTERNAL_H
#define POSITION_SENSOR_INTERNAL_H

#include "position_sensor.h"

typedef struct {
  bool valid;
  uint32_t diagnostic_flags;
  uint32_t transport_error_score;
  bool calibrated;
} PositionSensorDriverHealth_t;

typedef struct {
  PositionSensorStatus_t (*init)(void);
  PositionSensorStatus_t (*update_and_read)(PositionSensorSample_t *sample);
  PositionSensorStatus_t (*set_pole_pairs)(uint8_t pole_pairs);
  PositionSensorStatus_t (*zero_mechanical_position)(void);
  PositionSensorStatus_t (*set_electrical_offset)(float offset_rad);
  PositionSensorStatus_t (*get_electrical_offset)(float *offset_rad);
  PositionSensorStatus_t (*get_health)(PositionSensorDriverHealth_t *health);
} PositionSensorRuntimeOps_t;

typedef struct {
  PositionSensorStatus_t (*capture)(
      PositionSensorCalibrationSnapshot_t *snapshot);
  PositionSensorStatus_t (*restore)(
      const PositionSensorCalibrationSnapshot_t *snapshot);
  PositionSensorStatus_t (*clear)(void);
  PositionSensorStatus_t (*set_valid)(bool valid);
} PositionSensorCalibrationOps_t;

typedef struct {
  PositionSensorStatus_t (*read)(PositionSensorRawCalibrationState_t *state);
  PositionSensorStatus_t (*prepare_clockwise)(void);
  PositionSensorStatus_t (*set_direction_and_rebase)(
      PositionSensorDirection_t direction);
  PositionSensorStatus_t (*commit_pole_pairs)(uint8_t pole_pairs);
  PositionSensorStatus_t (*commit_offset_and_lut)(
      int32_t offset_counts,
      const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]);
} PositionSensorRawCalibrationOps_t;

/* Private adapter contract. Public callers never depend on a driver vtable. */
typedef struct {
  PositionSensorDescriptor_t descriptor;
  PositionSensorRuntimeOps_t runtime;
  PositionSensorCalibrationOps_t calibration;
  PositionSensorRawCalibrationOps_t raw_calibration;
} PositionSensorAdapter_t;

const PositionSensorAdapter_t *PositionSensor_SelectAdapter(void);
const PositionSensorAdapter_t *PositionSensorMt6816_GetAdapter(void);
const PositionSensorAdapter_t *PositionSensorTmr3109_GetAdapter(void);
const PositionSensorAdapter_t *PositionSensorHall_GetAdapter(void);
const PositionSensorAdapter_t *PositionSensorAbz_GetAdapter(void);

#endif /* POSITION_SENSOR_INTERNAL_H */
