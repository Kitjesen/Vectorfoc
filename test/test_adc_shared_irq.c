// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0

#include "stm32g4xx_it.h"
#include "stm32g4xx_hal.h"

#include <assert.h>
#include <stdio.h>

static ADC_TypeDef s_adc1_regs;
static ADC_TypeDef s_adc2_regs;
static unsigned s_adc1_irq_count;
static unsigned s_adc2_irq_count;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
FDCAN_HandleTypeDef hfdcan1;
TIM_HandleTypeDef htim1;
PCD_HandleTypeDef hpcd_USB_FS;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_tim3_ch2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim17;

void Emergency_DisableBridgeOutputs(void) {}
void Emergency_Shutdown(void) {}
void Error_Handler(void) {}
void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *hfdcan) { (void)hfdcan; }
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) { (void)htim; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma) { (void)hdma; }
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd) { (void)hpcd; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart) { (void)huart; }

void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    s_adc1_irq_count++;
  } else if (hadc == &hadc2) {
    s_adc2_irq_count++;
  } else {
    assert(0);
  }
}

static void reset_irq_state(void) {
  s_adc1_regs.ISR = 0u;
  s_adc1_regs.IER = 0u;
  s_adc2_regs.ISR = 0u;
  s_adc2_regs.IER = 0u;
  s_adc1_irq_count = 0u;
  s_adc2_irq_count = 0u;
  hadc1.Instance = &s_adc1_regs;
  hadc2.Instance = &s_adc2_regs;
}

static void test_services_both_adc_handles_when_sources_are_pending(void) {
  reset_irq_state();
  s_adc1_regs.ISR = ADC_FLAG_JEOS;
  s_adc1_regs.IER = ADC_IT_JEOS;
  s_adc2_regs.ISR = ADC_FLAG_OVR;
  s_adc2_regs.IER = ADC_IT_OVR;

  ADC1_2_IRQHandler();

  assert(s_adc1_irq_count == 1u);
  assert(s_adc2_irq_count == 1u);
}

static void test_skips_adc2_when_no_interrupt_source_is_enabled(void) {
  reset_irq_state();
  s_adc1_regs.ISR = ADC_FLAG_JEOS;
  s_adc1_regs.IER = ADC_IT_JEOS;
  s_adc2_regs.ISR = ADC_FLAG_OVR;
  s_adc2_regs.IER = 0u;

  ADC1_2_IRQHandler();

  assert(s_adc1_irq_count == 1u);
  assert(s_adc2_irq_count == 0u);
}

static void test_skips_handles_without_pending_flags(void) {
  reset_irq_state();
  s_adc1_regs.ISR = ADC_FLAG_JEOS;
  s_adc1_regs.IER = 0u;
  s_adc2_regs.ISR = 0u;
  s_adc2_regs.IER = ADC_IT_OVR;

  ADC1_2_IRQHandler();

  assert(s_adc1_irq_count == 0u);
  assert(s_adc2_irq_count == 0u);
}

int main(void) {
  test_services_both_adc_handles_when_sources_are_pending();
  test_skips_adc2_when_no_interrupt_source_is_enabled();
  test_skips_handles_without_pending_flags();
  puts("ADC shared IRQ tests: PASS");
  return 0;
}
