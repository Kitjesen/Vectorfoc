#include "encoder_calibration_settings.h"
#include "param_encoder_calibration_internal.h"
#include "position_sensor_internal.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                      \
  do {                                                                        \
    if (!(condition)) {                                                       \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);            \
      return 1;                                                               \
    }                                                                         \
  } while (0)

static unsigned int s_init_calls;
static unsigned int s_restore_calls;
static unsigned int s_capture_calls;
static unsigned int s_clear_calls;
static PositionSensorCalibrationSnapshot_t s_driver_calibration;

static PositionSensorStatus_t Fake_Init(void) {
  ++s_init_calls;
  memset(&s_driver_calibration, 0, sizeof(s_driver_calibration));
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_Capture(
    PositionSensorCalibrationSnapshot_t *snapshot) {
  ++s_capture_calls;
  *snapshot = s_driver_calibration;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_Restore(
    const PositionSensorCalibrationSnapshot_t *snapshot) {
  ++s_restore_calls;
  s_driver_calibration = *snapshot;
  return POSITION_SENSOR_STATUS_OK;
}

static PositionSensorStatus_t Fake_Clear(void) {
  ++s_clear_calls;
  memset(&s_driver_calibration, 0, sizeof(s_driver_calibration));
  return POSITION_SENSOR_STATUS_OK;
}

static const PositionSensorAdapter_t s_fake_adapter = {
    .descriptor =
        {
            .name = "fake-persistent-sensor",
            .capabilities = POSITION_SENSOR_CAP_PERSISTENT_CALIBRATION,
        },
    .runtime =
        {
            .init = Fake_Init,
        },
    .calibration =
        {
            .capture = Fake_Capture,
            .restore = Fake_Restore,
            .clear = Fake_Clear,
        },
};

const PositionSensorAdapter_t *PositionSensor_SelectAdapter(void) {
  return &s_fake_adapter;
}

static int CalibrationMatchesFlash(const FlashParamData *flash_data) {
  if (s_driver_calibration.valid !=
      (flash_data->encoder_calib_valid == 1U)) {
    return 0;
  }
  return memcmp(s_driver_calibration.offset_lut,
                flash_data->encoder_offset_lut,
                sizeof(s_driver_calibration.offset_lut)) == 0;
}

int main(void) {
  FlashParamData restored_image = {0};
  FlashParamData captured_image = {0};

  restored_image.encoder_calib_valid = 1U;
  for (unsigned int i = 0U; i < PARAM_ENCODER_CALIBRATION_LUT_SIZE; ++i) {
    restored_image.encoder_offset_lut[i] = (int16_t)((int)i - 64);
  }

  EncoderCalibrationSettings_InstallAdapter();
  CHECK(!PositionSensor_IsInitialized());

  /* Production initializes the concrete driver before restoring Flash because
   * driver init clears its runtime calibration fields. */
  CHECK(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);
  CHECK(s_init_calls == 1U);
  ParamEncoderCalibration_Restore(&restored_image);
  CHECK(s_restore_calls == 1U);
  CHECK(CalibrationMatchesFlash(&restored_image));

  ParamEncoderCalibration_Collect(&captured_image);
  CHECK(s_capture_calls == 1U);
  CHECK(captured_image.encoder_calib_valid == 1U);
  CHECK(memcmp(captured_image.encoder_offset_lut,
               restored_image.encoder_offset_lut,
               sizeof(captured_image.encoder_offset_lut)) == 0);

  ParamEncoderCalibration_Clear();
  CHECK(s_clear_calls == 1U);
  CHECK(!s_driver_calibration.valid);
  for (unsigned int i = 0U; i < POSITION_SENSOR_CALIBRATION_LUT_SIZE; ++i) {
    CHECK(s_driver_calibration.offset_lut[i] == 0);
  }

  puts("Encoder calibration settings startup-order test PASSED");
  return 0;
}
