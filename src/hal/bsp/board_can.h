// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "fdcan.h"
#include "main.h"
#include "protocol_messages.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_CAN_Init(void);
bool BSP_CAN_SendFrame(const CAN_Frame *frame);

#ifdef __cplusplus
}
#endif
#endif /* BSP_CAN_H */
