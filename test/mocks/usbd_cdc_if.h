// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#include <stdint.h>

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);

#endif /* USBD_CDC_IF_H */
