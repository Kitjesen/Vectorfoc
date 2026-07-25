// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef TEST_MOCK_VOFA_H
#define TEST_MOCK_VOFA_H

#include <stdbool.h>
#include <stdint.h>

bool Vofa_QueueReceive(const uint8_t *buf, uint16_t len);
void Vofa_ReportScheduledSaveResult(bool succeeded);
void Vofa_ReportScheduledSaveFailed(void);
void Vofa_OnTransmitComplete(void);

#endif /* TEST_MOCK_VOFA_H */
