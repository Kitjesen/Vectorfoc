#ifndef TEST_BOOT_HAL_H
#define TEST_BOOT_HAL_H

#include <stdint.h>

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }

#endif
