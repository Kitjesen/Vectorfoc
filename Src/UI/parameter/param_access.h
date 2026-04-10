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
 * @file param_access.h
 * @brief param - paramAPI
 *
 * :
 *   - safetyparam
 *   -
 *   - Flash
 *
 * :
 *   // param
 *   float Rs;
 *   Param_ReadFloat(PARAM_MOTOR_RS, &Rs);
 *
 *   // param
 *   Param_WriteFloat(PARAM_MOTOR_RS, 0.8f);
 *
 *   //
 *   Param_SaveToFlash();
 */
#ifndef PARAM_ACCESS_H
#define PARAM_ACCESS_H
#include "param_table.h"
/**
 * @brief param
 * @param index param
 * @param data output
 * @param type output (NULL)
 * @return ParamResult
 */
ParamResult Param_Read(uint16_t index, void *data, ParamType *type);
/**
 * @brief param
 * @param index param
 * @param data input
 * @return ParamResult
 */
ParamResult Param_Write(uint16_t index, const void *data);
/**
 * @brief floatparam ()
 * @param index param
 * @param value output
 * @return ParamResult
 */
ParamResult Param_ReadFloat(uint16_t index, float *value);
/**
 * @brief floatparam ()
 * @param index param
 * @param value input
 * @return ParamResult
 */
ParamResult Param_WriteFloat(uint16_t index, float value);
/**
 * @brief uint8param ()
 * @param index param
 * @param value output
 * @return ParamResult
 */
ParamResult Param_ReadUint8(uint16_t index, uint8_t *value);
/**
 * @brief uint8param ()
 * @param index param
 * @param value input
 * @return ParamResult
 */
ParamResult Param_WriteUint8(uint16_t index, uint8_t value);
/**
 * @brief paramFlash
 * @return ParamResult
 */
ParamResult Param_SaveToFlash(void);
/**
 * @brief Flashparam
 * @return ParamResult
 */
ParamResult Param_LoadFromFlash(void);
/**
 * @brief initparam（）
 * @return ParamResult
 */
ParamResult Param_SystemInitOnce(void);
/**
 * @brief param
 * @return ParamResult
 */
ParamResult Param_RestoreDefaults(void);
/**
 * @brief getparam
 * @param index param
 * @param entry outputparam
 * @return ParamResult
 */
ParamResult Param_GetInfo(uint16_t index, const ParamEntry **entry);
/**
 * @brief Flash (，ISRsafety)
 * @note set，actual Param_ProcessScheduledSave
 */
void Param_ScheduleSave(void);
/**
 * @brief  ()
 * @return true if save occurred, false otherwise
 */
bool Param_ProcessScheduledSave(void);
#endif /* PARAM_ACCESS_H */
