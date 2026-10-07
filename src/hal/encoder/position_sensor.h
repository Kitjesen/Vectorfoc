// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POSITION_SENSOR_H
#define POSITION_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POSITION_SENSOR_CALIBRATION_LUT_SIZE 128u

/** Capabilities reported by a position-sensor adapter. */
typedef uint32_t PositionSensorCapabilities_t;

enum {
  POSITION_SENSOR_CAP_ABSOLUTE = (1u << 0),
  POSITION_SENSOR_CAP_INCREMENTAL = (1u << 1),
  POSITION_SENSOR_CAP_COMMUTATION = (1u << 2),
  POSITION_SENSOR_CAP_INDEX = (1u << 3),
  POSITION_SENSOR_CAP_HEALTH = (1u << 4),
  /** Supports direction/pole-pair calibration using raw counts. */
  POSITION_SENSOR_CAP_RAW_DIRECTION_POLE = (1u << 5),
  /** Supports persistence of the generic calibrated/valid state. */
  POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION = (1u << 6),
  /** Supports the portable 128-entry linearity-correction snapshot. */
  POSITION_SENSOR_CAP_LINEARITY_CALIBRATION = (1u << 7),
  /** Deprecated compatibility spelling for persistent calibration. */
  POSITION_SENSOR_CAP_CALIBRATION = POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION
};

/** Stable result values for the public position-sensor boundary. */
typedef enum {
  POSITION_SENSOR_STATUS_OK = 0,
  POSITION_SENSOR_STATUS_UNSUPPORTED,
  POSITION_SENSOR_STATUS_NOT_INITIALIZED,
  POSITION_SENSOR_STATUS_NOT_READY,
  POSITION_SENSOR_STATUS_INVALID_ARGUMENT,
  POSITION_SENSOR_STATUS_IO_ERROR,
  POSITION_SENSOR_STATUS_INIT_FAILED
} PositionSensorStatus_t;

typedef enum {
  POSITION_SENSOR_DIRECTION_COUNTERCLOCKWISE = -1,
  POSITION_SENSOR_DIRECTION_CLOCKWISE = 1
} PositionSensorDirection_t;

/** One coherent real-time feedback frame. */
typedef struct {
  float position_rad;
  float mechanical_angle_rad;
  float velocity_rad_s;
  float electrical_angle_rad;
  int32_t native_raw;
} PositionSensorSample_t;

/** Generic diagnostic flags; concrete status enums remain adapter-private. */
enum {
  POSITION_SENSOR_DIAGNOSTIC_TRANSPORT = (1u << 0),
  POSITION_SENSOR_DIAGNOSTIC_FRAME_CHECK = (1u << 1),
  POSITION_SENSOR_DIAGNOSTIC_DEVICE = (1u << 2),
  POSITION_SENSOR_DIAGNOSTIC_SIGNAL_INVALID = (1u << 3)
};

typedef struct {
  bool valid;
  uint32_t consecutive_failures;
  uint32_t total_failures;
  uint32_t diagnostic_flags;
  uint32_t transport_error_score;
  bool calibrated;
} PositionSensorHealth_t;

typedef struct {
  bool valid;
  int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE];
} PositionSensorCalibrationSnapshot_t;

/** Raw state needed by direction, pole-pair, and LUT calibration only. */
typedef struct {
  int64_t shadow_count;
  int32_t count_in_cpr;
  uint32_t cpr;
  uint8_t pole_pairs;
  PositionSensorDirection_t direction;
  int32_t offset_counts;
} PositionSensorRawCalibrationState_t;

/** Read-only metadata for the adapter selected by board configuration. */
typedef struct {
  const char *name;
  PositionSensorCapabilities_t capabilities;
} PositionSensorDescriptor_t;

/** Return selected sensor metadata, or NULL for an unsupported mode. */
const PositionSensorDescriptor_t *PositionSensor_GetDescriptor(void);

/** Initialize the selected sensor once. Repeated successful calls are safe. */
PositionSensorStatus_t PositionSensor_Init(void);

/** Report whether the selected adapter completed initialization. */
bool PositionSensor_IsInitialized(void);

/**
 * Update hardware and publish one coherent frame.
 *
 * This is the only real-time entry point. It performs no allocation, sleeping,
 * or newly introduced blocking I/O beyond the selected driver's bounded update
 * operation. Publishing the complete frame uses one short interrupt-masked
 * critical section so task and higher-priority readers cannot observe a torn
 * structure.
 */
PositionSensorStatus_t
PositionSensor_UpdateAndRead(uint8_t pole_pairs,
                             PositionSensorSample_t *sample);

/** Copy the latest complete successful frame atomically without touching hardware. */
PositionSensorStatus_t
PositionSensor_GetLastSample(PositionSensorSample_t *sample);

PositionSensorStatus_t PositionSensor_SetPolePairs(uint8_t pole_pairs);
PositionSensorStatus_t PositionSensor_ZeroMechanicalPosition(void);
PositionSensorStatus_t PositionSensor_SetElectricalOffset(float offset_rad);
PositionSensorStatus_t PositionSensor_GetElectricalOffset(float *offset_rad);
PositionSensorStatus_t PositionSensor_GetHealth(PositionSensorHealth_t *health);

PositionSensorStatus_t PositionSensor_CaptureCalibration(
    PositionSensorCalibrationSnapshot_t *snapshot);
PositionSensorStatus_t PositionSensor_RestoreCalibration(
    const PositionSensorCalibrationSnapshot_t *snapshot);
PositionSensorStatus_t PositionSensor_ClearCalibration(void);
/** Update only the generic calibrated/valid flag, without changing the LUT. */
PositionSensorStatus_t PositionSensor_SetCalibrationValid(bool valid);

PositionSensorStatus_t
PositionSensor_RawCalibrationRead(PositionSensorRawCalibrationState_t *state);
PositionSensorStatus_t PositionSensor_RawCalibrationPrepareClockwise(void);
PositionSensorStatus_t PositionSensor_RawCalibrationSetDirectionAndRebase(
    PositionSensorDirection_t direction);
PositionSensorStatus_t
PositionSensor_RawCalibrationCommitPolePairs(uint8_t pole_pairs);
PositionSensorStatus_t PositionSensor_RawCalibrationCommitOffsetAndLut(
    int32_t offset_counts,
    const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_SENSOR_H */
