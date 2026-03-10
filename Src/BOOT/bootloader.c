#include "bootloader.h"
#include "stm32g4xx_hal.h"

#define BOOTLOADER_ADDR 0x1FFF0000U

void Boot_RequestUpgrade(void) {
  uint32_t sp = *(__IO uint32_t *)(BOOTLOADER_ADDR);
  uint32_t reset = *(__IO uint32_t *)(BOOTLOADER_ADDR + 4U);

  if (sp < 0x20000000U) {
    return;
  }

  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  for (uint32_t i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }

  __set_MSP(sp);
  __DSB();
  __ISB();
  ((void (*)(void))reset)();
}
