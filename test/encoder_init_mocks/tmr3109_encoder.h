#ifndef TEST_ENCODER_INIT_TMR3109_ENCODER_H
#define TEST_ENCODER_INIT_TMR3109_ENCODER_H

#include "mock_hal_types.h"

typedef struct {
  uint32_t marker;
} TMR3109_Handle_t;

extern TMR3109_Handle_t tmr3109_encoder_data;

void TMR3109_Init(TMR3109_Handle_t *encoder, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif /* TEST_ENCODER_INIT_TMR3109_ENCODER_H */
