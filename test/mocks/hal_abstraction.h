#ifndef TEST_MOCK_HAL_ABSTRACTION_H
#define TEST_MOCK_HAL_ABSTRACTION_H

#include <stdint.h>

#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

uint32_t HAL_GetSystemTick(void);
void HAL_Delay(uint32_t ms);

#endif
