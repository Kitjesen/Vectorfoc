#include "abz_encoder.h"
#include "board_config.h"
#include "hal_encoder.h"
#include "hall_encoder.h"
#include "motor.h"
#include "mt6816_encoder.h"
#include "tmr3109_encoder.h"

#include <stdint.h>
#include <stdio.h>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);              \
      return 1;                                                                \
    }                                                                          \
  } while (0)

SPI_HandleTypeDef test_encoder_spi;
GPIO_TypeDef *test_encoder_cs_port = (GPIO_TypeDef *)(uintptr_t)0x1234u;
MOTOR_DATA motor_data;
ENCODER_DATA encoder_data;
TMR3109_Handle_t tmr3109_encoder_data;

static uint32_t s_mt6816_init_calls;
static MT6816_Handle_t *s_mt6816_encoder;
static SPI_HandleTypeDef *s_mt6816_spi;
static GPIO_TypeDef *s_mt6816_cs_port;
static uint16_t s_mt6816_cs_pin;

static uint32_t s_tmr3109_init_calls;
static TMR3109_Handle_t *s_tmr3109_encoder;
static SPI_HandleTypeDef *s_tmr3109_spi;
static GPIO_TypeDef *s_tmr3109_cs_port;
static uint16_t s_tmr3109_cs_pin;
static uint32_t s_hall_init_calls;
static uint32_t s_abz_init_calls;

void MT6816_Init(MT6816_Handle_t *encoder, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin) {
  s_mt6816_init_calls++;
  s_mt6816_encoder = encoder;
  s_mt6816_spi = hspi;
  s_mt6816_cs_port = cs_port;
  s_mt6816_cs_pin = cs_pin;
}

void TMR3109_Init(TMR3109_Handle_t *encoder, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin) {
  s_tmr3109_init_calls++;
  s_tmr3109_encoder = encoder;
  s_tmr3109_spi = hspi;
  s_tmr3109_cs_port = cs_port;
  s_tmr3109_cs_pin = cs_pin;
}

void Hall_Init(void) { s_hall_init_calls++; }

void Abz_Init(void) { s_abz_init_calls++; }

int main(void) {
  HAL_Encoder_Interface_t unsupported_runtime_interface = {0};

  /* Encoder choice is now compile-time PositionSensor configuration.  The
   * legacy runtime-registration API must not report a registration it ignores. */
  CHECK(MHAL_Encoder_Register(&unsupported_runtime_interface) == -1);

#ifdef BOARD_XSTAR
#if HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_HALL
  CHECK(MHAL_Encoder_Init() == 0);
  CHECK(s_hall_init_calls == 1u);
  CHECK(s_abz_init_calls == 0u);
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_ABZ
  CHECK(MHAL_Encoder_Init() == 0);
  CHECK(s_abz_init_calls == 1u);
  CHECK(s_hall_init_calls == 0u);
#else
  CHECK(MHAL_Encoder_Init() == -1);
  CHECK(s_hall_init_calls == 0u);
  CHECK(s_abz_init_calls == 0u);
#endif
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_MT6816
  CHECK(MHAL_Encoder_Init() == 0);
  CHECK(s_mt6816_init_calls == 1u);
  CHECK(s_mt6816_encoder == &encoder_data);
  CHECK(s_mt6816_spi == &test_encoder_spi);
  CHECK(s_mt6816_cs_port == test_encoder_cs_port);
  CHECK(s_mt6816_cs_pin == HW_ENC_CS_PIN);
  CHECK(s_tmr3109_init_calls == 0u);
#elif HW_POSITION_SENSOR_MODE == HW_POSITION_SENSOR_TMR3109
  CHECK(MHAL_Encoder_Init() == 0);
  CHECK(s_tmr3109_init_calls == 1u);
  CHECK(s_tmr3109_encoder == &tmr3109_encoder_data);
  CHECK(s_tmr3109_spi == &test_encoder_spi);
  CHECK(s_tmr3109_cs_port == test_encoder_cs_port);
  CHECK(s_tmr3109_cs_pin == HW_ENC_CS_PIN);
  CHECK(s_mt6816_init_calls == 0u);
#else
  CHECK(MHAL_Encoder_Init() == -1);
  CHECK(s_mt6816_init_calls == 0u);
  CHECK(s_tmr3109_init_calls == 0u);
#endif

  puts("HAL encoder initialization tests PASSED");
  return 0;
}
