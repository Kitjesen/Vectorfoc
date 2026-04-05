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

/**
 * @file cmd_service.h
 * @brief
 */
#ifndef CMD_SERVICE_H
#define CMD_SERVICE_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void CmdService_Init(void);                   // init
void CmdService_Process(void);                //
void CmdService_SetReportEnable(bool enable); // setstate
#ifdef __cplusplus
}
#endif
#endif /* CMD_SERVICE_H */
