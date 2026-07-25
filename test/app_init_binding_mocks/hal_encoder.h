#ifndef TEST_APP_INIT_BINDING_MOCKS_HAL_ENCODER_H
#define TEST_APP_INIT_BINDING_MOCKS_HAL_ENCODER_H
#include <stdint.h>
int MHAL_Encoder_Init(void);
int MHAL_Encoder_Update(uint8_t pole_pairs);
#endif