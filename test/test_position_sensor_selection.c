#include "position_sensor.h"

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

int main(void) {
  const PositionSensorDescriptor_t *descriptor =
      PositionSensor_GetDescriptor();

#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_MT6816
  CHECK(descriptor != NULL);
  CHECK(strcmp(descriptor->name, "MT6816") == 0);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_ABSOLUTE) != 0u);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_HEALTH) != 0u);
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  CHECK(descriptor != NULL);
  CHECK(strcmp(descriptor->name, "TMR3109") == 0);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_ABSOLUTE) != 0u);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_HEALTH) != 0u);
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  CHECK(descriptor != NULL);
  CHECK(strcmp(descriptor->name, "Hall") == 0);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_COMMUTATION) != 0u);
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
  CHECK(descriptor != NULL);
  CHECK(strcmp(descriptor->name, "ABZ") == 0);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_INCREMENTAL) != 0u);
  CHECK((descriptor->capabilities & POSITION_SENSOR_CAP_INDEX) != 0u);
#else
  CHECK(descriptor == NULL);
  CHECK(PositionSensor_Init() == POSITION_SENSOR_STATUS_UNSUPPORTED);
  CHECK(!PositionSensor_IsInitialized());
  puts("Position sensor unsupported-selection test PASSED");
  return 0;
#endif

  CHECK(!PositionSensor_IsInitialized());
  CHECK(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);
  CHECK(PositionSensor_IsInitialized());
  CHECK(PositionSensor_GetDescriptor() == descriptor);
  CHECK(PositionSensor_Init() == POSITION_SENSOR_STATUS_OK);

  puts("Position sensor selection/init test PASSED");
  return 0;
}
