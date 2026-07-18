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

#include "app_comm_bootstrap.h"

#include "bsp_can.h"
#include "can_transport.h"
#include "manager.h"
#include "motor.h"
#include "safety_control.h"

static bool AppComm_ReportFaultCallback(uint32_t fault_bits, void *motor) {
  return Protocol_ReportFaultCallback(fault_bits, (MOTOR_DATA *)motor);
}

static ProtocolType AppComm_GetBootProtocol(void) {
  if (g_protocol_type <= PROTOCOL_MIT) {
    return (ProtocolType)g_protocol_type;
  }
  g_protocol_type = PROTOCOL_VECTOR;
  return PROTOCOL_VECTOR;
}

void AppComm_Bootstrap(void) {
  BSP_CAN_Init();
  CAN_Transport_Init();
  Protocol_RegisterTransport(CAN_Transport_GetInterface());
  Protocol_Init(AppComm_GetBootProtocol());
  Safety_RegisterFaultCallback(AppComm_ReportFaultCallback);
}
