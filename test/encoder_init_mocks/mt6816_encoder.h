#ifndef TEST_ENCODER_INIT_MT6816_ENCODER_H
#define TEST_ENCODER_INIT_MT6816_ENCODER_H

#include "mock_hal_types.h"

typedef struct MT6816_Handle_t {
  uint32_t marker;
} MT6816_Handle_t;

typedef MT6816_Handle_t ENCODER_DATA;

extern ENCODER_DATA encoder_data;

void MT6816_Init(MT6816_Handle_t *encoder, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif /* TEST_ENCODER_INIT_MT6816_ENCODER_H */
