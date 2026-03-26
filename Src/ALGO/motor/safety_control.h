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
 * @brief checkfault
 * @return true=fault，false=fault
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
