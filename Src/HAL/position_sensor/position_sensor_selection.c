#include "position_sensor_internal.h"

#include "board_config.h"

#include <stddef.h>

const PositionSensorAdapter_t *PositionSensor_SelectAdapter(void) {
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_MT6816
  return PositionSensorMt6816_GetAdapter();
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  return PositionSensorTmr3109_GetAdapter();
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  return PositionSensorHall_GetAdapter();
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
  return PositionSensorAbz_GetAdapter();
#else
  return NULL;
#endif
}
