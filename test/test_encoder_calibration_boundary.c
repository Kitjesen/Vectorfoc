#include "calib_encoder.h"
#include "position_sensor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);             \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static PositionSensorDescriptor_t s_descriptor;
static PositionSensorRawCalibrationState_t s_raw;
static PositionSensorDirection_t s_committed_direction;
static uint8_t s_committed_pole_pairs;
static bool s_calibration_valid;

const PositionSensorDescriptor_t *PositionSensor_GetDescriptor(void) {
  return &s_descriptor;
}

PositionSensorStatus_t
PositionSensor_RawCalibrationRead(PositionSensorRawCalibrationState_t *state) {
  *state = s_raw;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_RawCalibrationSetDirectionAndRebase(
    PositionSensorDirection_t direction) {
  s_committed_direction = direction;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t
PositionSensor_RawCalibrationCommitPolePairs(uint8_t pole_pairs) {
  s_committed_pole_pairs = pole_pairs;
  s_raw.pole_pairs = pole_pairs;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_RawCalibrationCommitOffsetAndLut(
    int32_t offset_counts,
    const int16_t offset_lut[POSITION_SENSOR_CALIBRATION_LUT_SIZE]) {
  (void)offset_counts;
  (void)offset_lut;
  return POSITION_SENSOR_STATUS_OK;
}

PositionSensorStatus_t PositionSensor_SetCalibrationValid(bool valid) {
  s_calibration_valid = valid;
  return POSITION_SENSOR_STATUS_OK;
}

void Control_InjectVoltage(MOTOR_DATA *motor, float vd, float vq, float angle) {
  (void)motor;
  (void)vd;
  (void)vq;
  (void)angle;
}

int MHAL_PWM_Brake(void) { return 0; }

void PID_clear(PidTypeDef *pid) { memset(pid, 0, sizeof(*pid)); }

static void ResetFake(void) {
  memset(&s_descriptor, 0, sizeof(s_descriptor));
  memset(&s_raw, 0, sizeof(s_raw));
  s_committed_direction = (PositionSensorDirection_t)0;
  s_committed_pole_pairs = 0u;
  s_calibration_valid = false;
}

static int TestLargeNativeCountUpdatesSingleSourceOfTruth(void) {
  MOTOR_DATA motor;
  DirectionPoleCalibContext context;
  const int64_t start_count = INT64_C(1234567890123);

  ResetFake();
  memset(&motor, 0, sizeof(motor));
  memset(&context, 0, sizeof(context));
  s_descriptor.capabilities = POSITION_SENSOR_CAP_RAW_DIRECTION_POLE |
                              POSITION_SENSOR_CAP_LINEARITY_CALIBRATION |
                              POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION;
  s_raw.cpr = 8388608u;
  s_raw.shadow_count = start_count + 4194304;
  motor.parameters.pole_pairs = 7;
  motor.state.Cs_State = CS_DIR_PP_END;
  context.start_count = start_count;

  CHECK(DirectionPoleCalib_Update(&motor, &context) == CALIB_SUCCESS);
  CHECK(s_committed_direction == POSITION_SENSOR_DIRECTION_CLOCKWISE);
  CHECK(s_committed_pole_pairs == 8u);
  CHECK(motor.parameters.pole_pairs == 8);
  CHECK(motor.params_updated);
  CHECK(motor.state.Cs_State == CS_ENCODER_START);
  return 0;
}

static int TestPersistentOnlySensorSkipsMechanicalCalibration(void) {
  MOTOR_DATA motor;
  DirectionPoleCalibContext context;

  ResetFake();
  memset(&motor, 0, sizeof(motor));
  memset(&context, 0, sizeof(context));
  s_descriptor.capabilities = POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION;
  motor.state.Cs_State = CS_DIR_PP_START;

  CHECK(DirectionPoleCalib_Update(&motor, &context) == CALIB_SUCCESS);
  CHECK(s_calibration_valid);
  CHECK(motor.state.Cs_State == CS_ENCODER_START);
  CHECK(s_committed_pole_pairs == 0u);
  return 0;
}

int main(void) {
  int failures = 0;
  failures += TestLargeNativeCountUpdatesSingleSourceOfTruth();
  failures += TestPersistentOnlySensorSkipsMechanicalCalibration();
  if (failures == 0) {
    puts("Encoder calibration boundary tests PASSED");
  }
  return failures == 0 ? 0 : 1;
}
