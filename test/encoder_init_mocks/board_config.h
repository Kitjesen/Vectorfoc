#ifndef TEST_ENCODER_INIT_BOARD_CONFIG_H
#define TEST_ENCODER_INIT_BOARD_CONFIG_H

#include "mock_hal_types.h"

#define HW_POSITION_SENSOR_HALL 1u
#define HW_POSITION_SENSOR_ABZ 2u
#define HW_POSITION_SENSOR_MT6816 3u
#define HW_POSITION_SENSOR_TMR3109 4u

#ifndef HW_POSITION_SENSOR_MODE
#define HW_POSITION_SENSOR_MODE HW_POSITION_SENSOR_MT6816
#endif

extern SPI_HandleTypeDef test_encoder_spi;
extern GPIO_TypeDef *test_encoder_cs_port;

#define HW_ENC_SPI test_encoder_spi
#define HW_ENC_CS_PORT test_encoder_cs_port
#define HW_ENC_CS_PIN 0x0040u

#endif /* TEST_ENCODER_INIT_BOARD_CONFIG_H */
