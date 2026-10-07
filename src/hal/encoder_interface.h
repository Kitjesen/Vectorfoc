#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#include "position_sensor.h"
#include <stdint.h>

typedef struct {
  float position_rad;
  float angle_rad;
  float velocity_rad;
  float elec_angle;
  int32_t raw_value;
} EncoderData;

int MHAL_Encoder_Init(void);
int MHAL_Encoder_Update(void);
PositionSensorStatus_t MHAL_Encoder_GetLastUpdateStatus(void);
void MHAL_Encoder_GetData(EncoderData *data);
float MHAL_Encoder_GetPosition(void);
float MHAL_Encoder_GetVelocity(void);
int MHAL_Encoder_SetOffset(float offset);
float MHAL_Encoder_GetOffset(void);
int MHAL_Encoder_ZeroPosition(void);

#endif
