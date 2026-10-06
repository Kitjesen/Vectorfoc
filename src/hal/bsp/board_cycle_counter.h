// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdint.h>

void DWT_Init(uint32_t cpu_freq_mhz);
void DWT_Delay(float delay_s);

#endif /* BSP_DWT_H */
