/**
 * @file fault_detection.h
 * @brief fault -
 * @version 2.0
 * @date 2026-01-23
 */
#ifndef FAULT_DETECTION_H
#define FAULT_DETECTION_H
#include "fault_def.h"
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief init
 * @param config configparam (NULL)
 */
void Detection_Init(const DetectionConfig *config);
/**
 * @brief  (state)
 * @param motor motor (void* )
 * @return fault
 */
uint32_t Detection_Check(void *motor);
/**
 * @brief fault (fault，20kHz)
 * @param motor motor
 * @return fault
 * @note fault，1μs
 */
uint32_t Detection_Check_Fast(void *motor);
/**
 * @brief fault (fault，200Hz)
 * @param motor motor
 * @return fault
 * @note voltage、temperature、、CANtimeout，3μs
 */
uint32_t Detection_Check_Slow(void *motor);
/**
 * @brief state ()
 */
void Detection_Reset(void);
/**
 * @brief getstate ()
 */
const DetectionState *Detection_GetState(void);
/**
 * @brief updateCAN ()
 */
void Detection_FeedWatchdog(uint32_t timestamp);
/**
 * @brief getconfig (param)
 * @return config ()
 */
DetectionConfig *Detection_GetConfig(void);
/**
 * @brief config (paraminit)
 */
extern DetectionConfig s_config;
#endif /* FAULT_DETECTION_H */
