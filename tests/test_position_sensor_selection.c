#include "position_sensor_internal.h"

#include <assert.h>
#include <stddef.h>

static const PositionSensorAdapter_t mt6816 = {
    .descriptor = {.name = "MT6816", .capabilities = POSITION_SENSOR_CAP_ABSOLUTE}};
static const PositionSensorAdapter_t tmr3109 = {
    .descriptor = {.name = "TMR3109", .capabilities = POSITION_SENSOR_CAP_ABSOLUTE}};

const PositionSensorAdapter_t *PositionSensorMt6816_GetAdapter(void) {
  return &mt6816;
}
const PositionSensorAdapter_t *PositionSensorTmr3109_GetAdapter(void) {
  return &tmr3109;
}
const PositionSensorAdapter_t *PositionSensorHall_GetAdapter(void) { return NULL; }
const PositionSensorAdapter_t *PositionSensorAbz_GetAdapter(void) { return NULL; }

int main(void) {
  const PositionSensorAdapter_t *selected = PositionSensor_SelectAdapter();
  assert(selected == &mt6816);
  assert((selected->descriptor.capabilities & POSITION_SENSOR_CAP_ABSOLUTE) != 0u);
  return 0;
}
