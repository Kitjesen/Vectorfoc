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
