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
 * @file safety_control.h
 * @brief safety - error
 * @version 4.0
 * @date 2026-01-28
 */
#ifndef SAFETY_CONTROL_H
#define SAFETY_CONTROL_H
#include "error_manager.h"
#include "fault_detection.h"
#include "fsm.h"
#include "motor.h"
/* ========== API ========== */
/**
 * @brief initsafety
 * @param config configparam（NULL=config）
 */
void Safety_Init(const SafetyConfig *config);
/**
 * @brief safety（period，20kHz）
 * @param motor motor
 * @param fsm state
 */
void Safety_Update(MOTOR_DATA *motor, StateMachine *fsm);
/**
 * @brief safety (20kHz)
 * @param motor motor
 * @param fsm state
 * @note fault，CPU1μs
 */
void Safety_Update_Fast(MOTOR_DATA *motor, StateMachine *fsm);
/**
 * @brief safety (200Hz)
 * @param motor motor
 * @param fsm state
 * @note fault，CPU3μs
 */
void Safety_Update_Slow(MOTOR_DATA *motor, StateMachine *fsm);
/**
 * @brief faultstate
 * @param fsm state
 */
void Safety_ClearFaults(StateMachine *fsm);
/**
 * @brief 检查是否有激活的故障
 * @return true = 当前有未清除的故障，false = 无故障
 */
bool Safety_HasActiveFault(void);
/**
 * @brief getfault（FaultBit）
 * @return fault
 */
uint32_t Safety_GetActiveFaultBits(void);
/**
 * @brief fault
 * @param callback
 */
void Safety_RegisterFaultCallback(SafetyFaultCallback callback);
/**
 * @brief getfault
 * @return fault (ms, from HAL_GetTick)
 */
uint32_t Safety_GetLastFaultTime(void);
#endif /* SAFETY_CONTROL_H */
