// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
#include "../Src/HAL/stm32g4/motor_adc.h"

#include <assert.h>
#include <stdint.h>

volatile uint16_t adc1_dma_value[adc1_samples][adc1_channel];
volatile uint16_t adc2_dma_value[adc2_samples][adc2_channel];

typedef struct {
  MotorAdcCurrentSample first;
  uint32_t index;
  uint32_t fail_at;
} CalibrationReaderContext;

static bool ReadCalibrationSample(void *opaque,
                                  MotorAdcCurrentSample *sample) {
  CalibrationReaderContext *context = opaque;
  if (context == NULL || sample == NULL || context->index == context->fail_at) {
    return false;
  }
  sample->phase_a = (uint16_t)(context->first.phase_a + context->index);
  sample->phase_b = (uint16_t)(context->first.phase_b + context->index);
  sample->phase_c = (uint16_t)(context->first.phase_c + context->index);
  ++context->index;
  return true;
}
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

static void TestOffsetCalibrationAveragesFreshSamples(void) {
  CalibrationReaderContext context = {
      .first = {.phase_a = 1000U, .phase_b = 2000U, .phase_c = 3000U},
      .index = 0U,
      .fail_at = UINT32_MAX,
  };
  MotorAdcCurrentOffsets result = {0};

  assert(MotorAdc_CalibrateOffsets(ReadCalibrationSample, &context, 3U,
                                   256U, 3840U, &result));
  assert(context.index == 3U);
  assert(result.phase_a == 1001.0f);
  assert(result.phase_b == 2001.0f);
  assert(result.phase_c == 3001.0f);
}

static void TestOffsetCalibrationFailureDoesNotPublishPartialResult(void) {
  CalibrationReaderContext context = {
      .first = {.phase_a = 1000U, .phase_b = 2000U, .phase_c = 3000U},
      .index = 0U,
      .fail_at = 2U,
  };
  MotorAdcCurrentOffsets result = {
      .phase_a = 11.0f,
      .phase_b = 22.0f,
      .phase_c = 33.0f,
  };

  assert(!MotorAdc_CalibrateOffsets(ReadCalibrationSample, &context, 3U,
                                    256U, 3840U, &result));
  assert(result.phase_a == 11.0f);
  assert(result.phase_b == 22.0f);
  assert(result.phase_c == 33.0f);
}

static void TestOffsetCalibrationRejectsOutOfRangeAverage(void) {
  CalibrationReaderContext context = {
      .first = {.phase_a = 100U, .phase_b = 2000U, .phase_c = 3000U},
      .index = 0U,
      .fail_at = UINT32_MAX,
  };
  MotorAdcCurrentOffsets result = {0};

  assert(!MotorAdc_CalibrateOffsets(ReadCalibrationSample, &context, 2U,
                                    256U, 3840U, &result));
  assert(!MotorAdc_CalibrateOffsets(NULL, &context, 2U,
                                    256U, 3840U, &result));
  assert(!MotorAdc_CalibrateOffsets(ReadCalibrationSample, &context, 0U,
                                    256U, 3840U, &result));
}
int main(void) {
  TestMedianUsesSnapshot();
  TestChannelBounds();
  return 0;
}
