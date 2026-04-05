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
 * @file    vofa.h
 * @brief   VectorStudio
 *
 * @details  VectorStudio :
 *
 *   ┌────────────────────────────────────────────────────┐
 *   │          USB-CDC  ()                 │
 *   ├────────────────  → PC ─────────────────────────┤
 *   │  1. : FireWater  (12ch × float)     │
 *   │  2. state: "key=value\n" (calibration/fault/)       │
 *   ├──────────────── PC →  ─────────────────────────┤
 *   │  3. : "key=value\n" (//)         │
 *   └────────────────────────────────────────────────────┘
 *
 *    ( VOFA+ FireWater):
 *     [float0][float1]...[float11][0x00 0x00 0x80 0x7F]
 *     48  (Little-Endian IEEE754) + 4  = 52
 *
 * @note  vofa.h  include，actual VectorStudio
 */
#ifndef VOFA_H
#define VOFA_H
#include "common.h"
#include "motor.h"
#include "motor_adc.h"
#include "mt6816_encoder.h"
#include "usart.h"
#include "usbd_cdc_if.h"
/* ============================================================================
 *   ( float → byte )
 * ============================================================================
 */
#define byte0(dw_temp) (*(char *)(&dw_temp))
#define byte1(dw_temp) (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp) (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp) (*((char *)(&dw_temp) + 3))
/* ============================================================================
 *  Scope  (ISR → Task )
 * ============================================================================
 */
#define SCOPE_BUFFER_SIZE 64 // 64 * 12 * 4 = 3 KB
#define SCOPE_CHANNELS 12
typedef struct {
  float data[SCOPE_BUFFER_SIZE][SCOPE_CHANNELS];
  volatile uint16_t head;
  volatile uint16_t tail;
} ScopeBuffer_t;
/* ============================================================================
 *   API (FireWater )
 * ============================================================================
 */
/** @brief ISR sample ( isr_foc.c , 1kHz) */
void Scope_Init(void);
void Scope_Update(void);
/** @brief Task  ( task_debug.c , 1kHz) */
void Scope_Process(void);
/*  (mode) */
void vofa_start(void);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);
void Vofa_Packet(void);
/* ============================================================================
 *  state API ( → VectorStudio)
 * ============================================================================
 */
/**
 * @brief   ( '\n')
 * @param  text   ()
 */
void Studio_SendText(const char *text);
/**
 * @brief   ( printf,  '\n')
 * @param  fmt
 * @param  ...  param
 */
void Studio_SendTextf(const char *fmt, ...);
/** @brief : "fw_version=X.Y.Z" */
void Studio_ReportVersion(void);
/** @brief  ( set_scope_enable ) */
bool Studio_IsScopeEnabled(void);
/** @brief calibration: "calib_step=N" / "calib_done=1" / "calib_error=MSG" */
void Studio_ReportCalibStatus(void);
/** @brief fault: "fault=CODE,SEVERITY"  "fault_clear=all" */
void Studio_ReportFaults(void);
/**
 * @brief  periodstate ( task_debug.c  ~10Hz )
 * @details state:
 *   - calibration → calib_step=N
 *   - calibrationdone → calib_done=1
 *   - fault → fault=CODE,SEVERITY
 *   - fault → fault_clear=all
 */
void Studio_PeriodicUpdate(void);
/* ============================================================================
 *   API (VectorStudio → )
 * ============================================================================
 */
/**
 * @brief   ( usbd_cdc_if.c )
 * @param  buf
 * @param  len
 */
void vofa_Receive(uint8_t *buf, uint16_t len);
#endif // VOFA_H
