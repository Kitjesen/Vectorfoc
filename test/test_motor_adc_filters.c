// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
#include "../Src/HAL/stm32g4/motor_adc.h"

#include <assert.h>
#include <stdint.h>

volatile uint16_t adc1_dma_value[adc1_samples][adc1_channel];
volatile uint16_t adc2_dma_value[adc2_samples][adc2_channel];

static void TestMedianUsesSnapshot(void) {
  static const uint16_t adc1_input[adc1_samples] = {9U, 1U, 5U, 3U, 7U};
  static const uint16_t adc2_input[adc2_samples] = {20U, 10U, 50U, 30U, 40U};

  for (uint16_t i = 0U; i < adc1_samples; ++i) {
    adc1_dma_value[i][adc1_ch1] = adc1_input[i];
  }
  for (uint16_t i = 0U; i < adc2_samples; ++i) {
    adc2_dma_value[i][adc2_ch12] = adc2_input[i];
  }

  assert(adc1_median_filter(adc1_ch1) == 5U);
  assert(adc2_median_filter(adc2_ch12) == 30U);

  for (uint16_t i = 0U; i < adc1_samples; ++i) {
    assert(adc1_dma_value[i][adc1_ch1] == adc1_input[i]);
  }
  for (uint16_t i = 0U; i < adc2_samples; ++i) {
    assert(adc2_dma_value[i][adc2_ch12] == adc2_input[i]);
  }
}

static void TestChannelBounds(void) {
  assert(adc1_median_filter(adc1_channel) == 0U);
  assert(adc1_avg_filter(adc1_channel) == 0U);
  assert(adc2_median_filter(adc2_channel) == 0U);
  assert(adc2_avg_filter(adc2_channel) == 0U);
}

int main(void) {
  TestMedianUsesSnapshot();
  TestChannelBounds();
  return 0;
}
