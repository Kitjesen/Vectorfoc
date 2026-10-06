// Copyright 2024-2026 VectorFOC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** VectorStudio USB interface. Wire formats stay in telemetry_vofa.c. */
#ifndef VOFA_H
#define VOFA_H
#include <stdint.h>

/** Capture one 12-channel sample in the FOC ISR. */
void Scope_Update(void);
/** Transmit queued samples from the USB task. */
void Scope_Process(void);
/** Publish the existing periodic state, calibration and fault reports. */
void Studio_PeriodicUpdate(void);
/** Handle the existing text commands from the USB CDC receive callback. */
void vofa_Receive(uint8_t *buf, uint16_t len);
#endif /* VOFA_H */
