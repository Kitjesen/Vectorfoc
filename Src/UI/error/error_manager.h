/**
 * @file error_manager.h
 * @brief error - error
 * @version 3.0
 * @date 2026-01-28
 *
 * @details ：
 * - error
 * - error（）
 * - error
 * - error
 * - error
 */
#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H
#include "error_types.h"
#include "error_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/* ========== configparam（error_config.h） ========== */
#ifndef ERROR_HISTORY_SIZE
#define ERROR_HISTORY_SIZE 32  /**< error */
#endif
#ifndef ERROR_MAX_CALLBACKS
#define ERROR_MAX_CALLBACKS 4  /**<  */
#endif
/* ========== error ========== */
typedef struct {
    uint32_t total_count;      /**< error */
    uint32_t info_count;       /**<  */
    uint32_t warning_count;    /**< warning */
    uint32_t minor_count;      /**< fault */
    uint32_t major_count;      /**< fault */
    uint32_t critical_count;   /**< fault */
    uint32_t last_error_time;  /**< error */
} ErrorStatistics;
/* ========== API ========== */
/**
 * @brief initerror
 * @note
 */
void ErrorManager_Init(void);
/**
 * @brief error（，）
 * @param error_code error
 * @param message error（NULL）
 * @param file
 * @param line
 */
void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line);
/**
 * @brief error（）
 * @param error_code error
 * @param message error（NULL）
 */
void ErrorManager_Report(uint32_t error_code, const char *message);
/**
 * @brief error（）
 */
#define ERROR_REPORT(code, msg) \
    ErrorManager_ReportFull(code, msg, __FILE__, __LINE__)
/**
 * @brief error（）
 */
#define ERROR_REPORT_CODE(code) \
    ErrorManager_ReportFull(code, NULL, __FILE__, __LINE__)
/* ========== API ========== */
/**
 * @brief geterror
 * @param index （0=，）
 * @return error，NULL
 */
const ErrorRecord *ErrorManager_GetHistory(uint16_t index);
/**
 * @brief get
 * @return
 */
uint16_t ErrorManager_GetHistoryCount(void);
/**
 * @brief geterror
 * @return
 */
const ErrorStatistics *ErrorManager_GetStatistics(void);
/**
 * @brief checkfault
 * @return true=fault，false=fault
 */
bool ErrorManager_HasActiveFault(void);
/**
 * @brief getfault（）
 * @return fault
 */
uint32_t ErrorManager_GetActiveFaults(void);
/**
 * @brief checkfault
 * @param domain error
 * @return true=fault，false=fault
 */
bool ErrorManager_HasFaultInDomain(ErrorDomain domain);
/* ========== API ========== */
/**
 * @brief error
 */
void ErrorManager_ClearAll(void);
/**
 * @brief error
 * @param domain error
 */
void ErrorManager_ClearDomain(ErrorDomain domain);
/**
 * @brief fault
 * @note ，
 */
void ErrorManager_ClearActiveFaults(void);
/* ========== API ========== */
/**
 * @brief error
 * @param callback
 * @return true=，false=（）
 */
bool ErrorManager_RegisterCallback(ErrorCallback callback);
/**
 * @brief error
 * @param callback
 * @return true=，false=（）
 */
bool ErrorManager_UnregisterCallback(ErrorCallback callback);
/* ========== API ========== */
/**
 * @brief geterror
 * @param domain error
 * @return
 */
const char *ErrorManager_GetDomainName(ErrorDomain domain);
/**
 * @brief get
 * @param severity
 * @return
 */
const char *ErrorManager_GetSeverityName(ErrorSeverity severity);
/**
 * @brief error
 * @param error_code error
 * @param buffer output
 * @param buffer_size
 * @return
 */
int ErrorManager_FormatError(uint32_t error_code, char *buffer, size_t buffer_size);
#ifdef __cplusplus
}
#endif
#endif /* ERROR_MANAGER_H */
