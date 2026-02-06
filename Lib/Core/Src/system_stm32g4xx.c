/**
 * @file    system_stm32g4xx.c
 * @author  MCD Application Team
 * @brief   CMSIS Cortex-M4 system source file
 *
 *          Provides SystemInit() and SystemCoreClockUpdate().
 *          Default clock after reset: HSI 16MHz.
 *          HSE_VALUE = 24MHz (defined in stm32g4xx_hal.h or here).
 *
 * Copyright (c) 2019 STMicroelectronics. All rights reserved.
 */

#include "stm32g4xx.h"

#if !defined(HSE_VALUE)
  #define HSE_VALUE 24000000U
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE 16000000U
#endif

/* Vector table relocation (uncomment if needed) */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
  #if defined(VECT_TAB_SRAM)
    #define VECT_TAB_BASE_ADDRESS SRAM_BASE
  #else
    #define VECT_TAB_BASE_ADDRESS FLASH_BASE
  #endif
  #define VECT_TAB_OFFSET 0x00000000U
#endif

uint32_t SystemCoreClock = HSI_VALUE;

const uint8_t AHBPrescTable[16] = {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

/**
 * @brief  Setup the microcontroller system (FPU, vector table).
 *         Called from startup before main().
 */
void SystemInit(void) {
  /* Enable FPU (CP10/CP11 full access) */
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
  #endif

  #if defined(USER_VECT_TAB_ADDRESS)
    SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
  #endif
}

/**
 * @brief  Update SystemCoreClock from current RCC register values.
 */
void SystemCoreClockUpdate(void) {
  uint32_t tmp, pllvco, pllr, pllsource, pllm;

  switch (RCC->CFGR & RCC_CFGR_SWS) {
    case 0x04: /* HSI */
      SystemCoreClock = HSI_VALUE;
      break;

    case 0x08: /* HSE */
      SystemCoreClock = HSE_VALUE;
      break;

    case 0x0C: /* PLL */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
      pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1U;
      if (pllsource == 0x02UL) {
        pllvco = (HSI_VALUE / pllm);
      } else {
        pllvco = (HSE_VALUE / pllm);
      }
      pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1U) * 2U;
      SystemCoreClock = pllvco / pllr;
      break;

    default:
      break;
  }

  /* Apply AHB prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  SystemCoreClock >>= tmp;
}
