// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "board_cycle_counter.h"
#include "main.h"

static uint32_t s_cpu_freq_hz;

void DWT_Init(uint32_t cpu_freq_mhz) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_cpu_freq_hz = cpu_freq_mhz * 1000000u;
}

void DWT_Delay(float delay_s) {
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < delay_s * (float)s_cpu_freq_hz) {
  }
}
