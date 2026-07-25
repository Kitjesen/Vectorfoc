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
 * @file error_manager.c
 * @brief error
 * @version 3.0
 */
#include "error_manager.h"
#include "bsp_log.h"
#include "platform.h"
#include <string.h>
#include <stdio.h>
/* ==========  ========== */
typedef struct {
    ErrorRecord history[ERROR_HISTORY_SIZE];  /**< error */
    uint16_t write_index;                     /**<  */
    uint16_t count;                           /**<  */
    ErrorStatistics stats;                    /**<  */
    uint32_t active_faults;                   /**< fault */
    ErrorCallback callbacks[ERROR_MAX_CALLBACKS]; /**<  */
    uint8_t callback_count;                   /**<  */
    bool initialized;                         /**< init */
} ErrorManagerContext;
static ErrorManagerContext s_ctx = {0};
static ErrorRecord s_history_snapshot;
static ErrorStatistics s_statistics_snapshot;
/* ==========  ========== */
static void HandleErrorBySeverity(uint32_t error_code, ErrorSeverity severity);
static void InvokeCallbacks(const ErrorRecord *record,
                            const ErrorCallback *callbacks,
                            uint8_t callback_count);
static uint32_t GetSystemTick(void);
static void EnsureInitialized(void);
/* ==========  ========== */
static const char *DOMAIN_NAMES[] = {
    "SYSTEM",
    "HARDWARE",
    "MOTOR",
    "SAFETY",
    "COMMUNICATION",
    "CALIBRATION",
    "PARAMETER",
    "APPLICATION",
};
/* ==========  ========== */
static const char *SEVERITY_NAMES[] = {
    "INFO",
    "WARNING",
    "MINOR",
    "MAJOR",
    "CRITICAL",
};
/* ========== API ========== */
void ErrorManager_Init(void) {
    CRITICAL_SECTION_BEGIN();
    memset(&s_ctx, 0, sizeof(ErrorManagerContext));
    s_ctx.initialized = true;
    CRITICAL_SECTION_END();
    LOGINFO("[ErrorMgr] Error Manager initialized");
}
void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line) {
    EnsureInitialized();
    if (error_code == ERROR_NONE) {
        return;
    }
    // 1. error
    ErrorSeverity severity = ERROR_GET_SEVERITY(error_code);
    ErrorDomain domain = ERROR_GET_DOMAIN(error_code);
    ErrorRecord record = {
        .error_code = error_code,
        .timestamp = GetSystemTick(),
        .message = message,
        .file = file,
        .line = line,
    };
    ErrorCallback callbacks[ERROR_MAX_CALLBACKS] = {0};
    uint8_t callback_count = 0;

    CRITICAL_SECTION_BEGIN();
    s_ctx.history[s_ctx.write_index] = record;
    s_ctx.write_index = (s_ctx.write_index + 1) % ERROR_HISTORY_SIZE;
    if (s_ctx.count < ERROR_HISTORY_SIZE) {
        s_ctx.count++;
    }
    // 3. update
    s_ctx.stats.total_count++;
    s_ctx.stats.last_error_time = record.timestamp;
    switch (severity) {
        case ERROR_SEVERITY_INFO:     s_ctx.stats.info_count++;     break;
        case ERROR_SEVERITY_WARNING:  s_ctx.stats.warning_count++;  break;
        case ERROR_SEVERITY_MINOR:    s_ctx.stats.minor_count++;    break;
        case ERROR_SEVERITY_MAJOR:    s_ctx.stats.major_count++;    break;
        case ERROR_SEVERITY_CRITICAL: s_ctx.stats.critical_count++; break;
    }
    // 4. updatefault（MAJOR）
    if (severity >= ERROR_SEVERITY_MAJOR) {
        s_ctx.active_faults |= (1u << domain);
    }
    callback_count = s_ctx.callback_count;
    memcpy(callbacks, s_ctx.callbacks,
           callback_count * sizeof(callbacks[0]));
    CRITICAL_SECTION_END();

    // 5. output
    char err_str[128];
    ErrorManager_FormatError(error_code, err_str, sizeof(err_str));
    if (message != NULL && file != NULL) {
        LOGERROR("[ErrorMgr] %s | %s | %s:%lu", err_str, message, file, line);
    } else if (message != NULL) {
        LOGERROR("[ErrorMgr] %s | %s", err_str, message);
    } else {
        LOGERROR("[ErrorMgr] %s", err_str);
    }
    // 6.
    InvokeCallbacks(&record, callbacks, callback_count);
    // 7.
    HandleErrorBySeverity(error_code, severity);
}
void ErrorManager_Report(uint32_t error_code, const char *message) {
    ErrorManager_ReportFull(error_code, message, NULL, 0);
}
/* ========== API ========== */
const ErrorRecord *ErrorManager_GetHistory(uint16_t index) {
    const ErrorRecord *result = NULL;
    CRITICAL_SECTION_BEGIN();
    if (index >= s_ctx.count) {
    } else {
        uint16_t actual_index;
        if (s_ctx.count < ERROR_HISTORY_SIZE) {
            actual_index = (s_ctx.count - 1 - index);
        } else {
            actual_index =
                (s_ctx.write_index + ERROR_HISTORY_SIZE - 1 - index) %
                ERROR_HISTORY_SIZE;
        }
        s_history_snapshot = s_ctx.history[actual_index];
        result = &s_history_snapshot;
    }
    CRITICAL_SECTION_END();
    return result;
}
uint16_t ErrorManager_GetHistoryCount(void) {
    uint16_t count;
    CRITICAL_SECTION_BEGIN();
    count = s_ctx.count;
    CRITICAL_SECTION_END();
    return count;
}
const ErrorStatistics *ErrorManager_GetStatistics(void) {
    CRITICAL_SECTION_BEGIN();
    s_statistics_snapshot = s_ctx.stats;
    CRITICAL_SECTION_END();
    return &s_statistics_snapshot;
}
bool ErrorManager_HasActiveFault(void) {
    bool has_fault;
    CRITICAL_SECTION_BEGIN();
    has_fault = (s_ctx.active_faults != 0);
    CRITICAL_SECTION_END();
    return has_fault;
}
uint32_t ErrorManager_GetActiveFaults(void) {
    uint32_t faults;
    CRITICAL_SECTION_BEGIN();
    faults = s_ctx.active_faults;
    CRITICAL_SECTION_END();
    return faults;
}
bool ErrorManager_HasFaultInDomain(ErrorDomain domain) {
    bool has_fault;
    CRITICAL_SECTION_BEGIN();
    has_fault = (s_ctx.active_faults & (1u << domain)) != 0;
    CRITICAL_SECTION_END();
    return has_fault;
}
/* ========== API ========== */
void ErrorManager_ClearAll(void) {
    CRITICAL_SECTION_BEGIN();
    s_ctx.write_index = 0;
    s_ctx.count = 0;
    s_ctx.active_faults = 0;
    memset(&s_ctx.stats, 0, sizeof(ErrorStatistics));
    CRITICAL_SECTION_END();
    LOGINFO("[ErrorMgr] All errors cleared");
}
void ErrorManager_ClearDomain(ErrorDomain domain) {
    CRITICAL_SECTION_BEGIN();
    s_ctx.active_faults &= ~(1u << domain);
    CRITICAL_SECTION_END();
}
void ErrorManager_ClearActiveFaults(void) {
    CRITICAL_SECTION_BEGIN();
    s_ctx.active_faults = 0;
    CRITICAL_SECTION_END();
    LOGINFO("[ErrorMgr] Active faults cleared");
}
/* ========== API ========== */
bool ErrorManager_RegisterCallback(ErrorCallback callback) {
    if (callback == NULL) {
        return false;
    }
    bool registered = false;
    CRITICAL_SECTION_BEGIN();
    if (s_ctx.callback_count >= ERROR_MAX_CALLBACKS) {
    } else {
    for (uint8_t i = 0; i < s_ctx.callback_count; i++) {
        if (s_ctx.callbacks[i] == callback) {
                goto register_done;
        }
    }
    s_ctx.callbacks[s_ctx.callback_count++] = callback;
        registered = true;
    }
register_done:
    CRITICAL_SECTION_END();
    return registered;
}
bool ErrorManager_UnregisterCallback(ErrorCallback callback) {
    bool removed = false;
    CRITICAL_SECTION_BEGIN();
    for (uint8_t i = 0; i < s_ctx.callback_count; i++) {
        if (s_ctx.callbacks[i] == callback) {
            for (uint8_t j = i; j < s_ctx.callback_count - 1; j++) {
                s_ctx.callbacks[j] = s_ctx.callbacks[j + 1];
            }
            s_ctx.callback_count--;
            s_ctx.callbacks[s_ctx.callback_count] = NULL;
            removed = true;
            break;
        }
    }
    CRITICAL_SECTION_END();
    return removed;
}
/* ========== API ========== */
const char *ErrorManager_GetDomainName(ErrorDomain domain) {
    if (domain < sizeof(DOMAIN_NAMES) / sizeof(DOMAIN_NAMES[0])) {
        return DOMAIN_NAMES[domain];
    }
    return "UNKNOWN";
}
const char *ErrorManager_GetSeverityName(ErrorSeverity severity) {
    if (severity < sizeof(SEVERITY_NAMES) / sizeof(SEVERITY_NAMES[0])) {
        return SEVERITY_NAMES[severity];
    }
    return "UNKNOWN";
}
int ErrorManager_FormatError(uint32_t error_code, char *buffer, size_t buffer_size) {
    ErrorSeverity severity = ERROR_GET_SEVERITY(error_code);
    ErrorDomain domain = ERROR_GET_DOMAIN(error_code);
    uint8_t subsys = ERROR_GET_SUBSYS(error_code);
    uint16_t code = ERROR_GET_CODE(error_code);
    return snprintf(buffer, buffer_size, "[%s][%s] 0x%08lX (subsys=%u, code=%u)",
                    ErrorManager_GetSeverityName(severity),
                    ErrorManager_GetDomainName(domain),
                    (unsigned long)error_code, subsys, code);
}
/* ==========  ========== */
static void HandleErrorBySeverity(uint32_t error_code, ErrorSeverity severity) {
    switch (severity) {
        case ERROR_SEVERITY_INFO:
            // ，
            break;
        case ERROR_SEVERITY_WARNING:
            // ：LED、output
            break;
        case ERROR_SEVERITY_MINOR:
            // running（）
            break;
        case ERROR_SEVERITY_MAJOR:
            // （state）
            break;
        case ERROR_SEVERITY_CRITICAL:
#ifndef TEST_ENV
            __disable_irq();
            HAL_NVIC_SystemReset();
            for (;;) {
            }
#endif
            break;
    }
}
static void InvokeCallbacks(const ErrorRecord *record,
                            const ErrorCallback *callbacks,
                            uint8_t callback_count) {
    for (uint8_t i = 0; i < callback_count; i++) {
        if (callbacks[i] != NULL) {
            callbacks[i](record);
        }
    }
}
static void EnsureInitialized(void) {
    bool initialized_now = false;
    CRITICAL_SECTION_BEGIN();
    if (!s_ctx.initialized) {
        memset(&s_ctx, 0, sizeof(s_ctx));
        s_ctx.initialized = true;
        initialized_now = true;
    }
    CRITICAL_SECTION_END();
    if (initialized_now) {
        LOGINFO("[ErrorMgr] Error Manager initialized");
    }
}
static uint32_t GetSystemTick(void) {
    // HAL
    extern uint32_t HAL_GetTick(void);
    return HAL_GetTick();
}
