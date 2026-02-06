/**
 * @file error_manager.c
 * @brief 统一错误管理器实现
 * @version 3.0
 */

#include "error_manager.h"
#include "bsp_log.h"
#include <string.h>
#include <stdio.h>

/* ========== 内部数据结构 ========== */
typedef struct {
    ErrorRecord history[ERROR_HISTORY_SIZE];  /**< 错误历史记录 */
    uint16_t write_index;                     /**< 写入索引 */
    uint16_t count;                           /**< 记录数量 */
    
    ErrorStatistics stats;                    /**< 统计信息 */
    uint32_t active_faults;                   /**< 激活的故障位掩码 */
    
    ErrorCallback callbacks[ERROR_MAX_CALLBACKS]; /**< 回调函数数组 */
    uint8_t callback_count;                   /**< 回调函数数量 */
    
    bool initialized;                         /**< 初始化标志 */
} ErrorManagerContext;

static ErrorManagerContext s_ctx = {0};

/* ========== 内部函数声明 ========== */
static void HandleErrorBySeverity(uint32_t error_code, ErrorSeverity severity);
static void InvokeCallbacks(const ErrorRecord *record);
static uint32_t GetSystemTick(void);

/* ========== 域名称字符串表 ========== */
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

/* ========== 严重级别字符串表 ========== */
static const char *SEVERITY_NAMES[] = {
    "INFO",
    "WARNING",
    "MINOR",
    "MAJOR",
    "CRITICAL",
};

/* ========== 核心API实现 ========== */

void ErrorManager_Init(void) {
    memset(&s_ctx, 0, sizeof(ErrorManagerContext));
    s_ctx.initialized = true;
    LOGINFO("[ErrorMgr] Error Manager initialized");
}

void ErrorManager_ReportFull(uint32_t error_code, const char *message,
                             const char *file, uint32_t line) {
    if (!s_ctx.initialized) {
        ErrorManager_Init();
    }
    
    if (error_code == ERROR_NONE) {
        return;
    }
    
    // 1. 解析错误码
    ErrorSeverity severity = ERROR_GET_SEVERITY(error_code);
    ErrorDomain domain = ERROR_GET_DOMAIN(error_code);
    
    // 2. 存储到历史记录
    ErrorRecord *record = &s_ctx.history[s_ctx.write_index];
    record->error_code = error_code;
    record->timestamp = GetSystemTick();
    record->message = message;
    record->file = file;
    record->line = line;
    
    s_ctx.write_index = (s_ctx.write_index + 1) % ERROR_HISTORY_SIZE;
    if (s_ctx.count < ERROR_HISTORY_SIZE) {
        s_ctx.count++;
    }
    
    // 3. 更新统计信息
    s_ctx.stats.total_count++;
    s_ctx.stats.last_error_time = record->timestamp;
    
    switch (severity) {
        case ERROR_SEVERITY_INFO:     s_ctx.stats.info_count++;     break;
        case ERROR_SEVERITY_WARNING:  s_ctx.stats.warning_count++;  break;
        case ERROR_SEVERITY_MINOR:    s_ctx.stats.minor_count++;    break;
        case ERROR_SEVERITY_MAJOR:    s_ctx.stats.major_count++;    break;
        case ERROR_SEVERITY_CRITICAL: s_ctx.stats.critical_count++; break;
    }
    
    // 4. 更新激活故障标志（MAJOR及以上）
    if (severity >= ERROR_SEVERITY_MAJOR) {
        s_ctx.active_faults |= (1u << domain);
    }
    
    // 5. 日志输出
    char err_str[128];
    ErrorManager_FormatError(error_code, err_str, sizeof(err_str));
    
    if (message != NULL && file != NULL) {
        LOGERROR("[ErrorMgr] %s | %s | %s:%lu", err_str, message, file, line);
    } else if (message != NULL) {
        LOGERROR("[ErrorMgr] %s | %s", err_str, message);
    } else {
        LOGERROR("[ErrorMgr] %s", err_str);
    }
    
    // 6. 调用回调函数
    InvokeCallbacks(record);
    
    // 7. 按严重级别处理
    HandleErrorBySeverity(error_code, severity);
}

void ErrorManager_Report(uint32_t error_code, const char *message) {
    ErrorManager_ReportFull(error_code, message, NULL, 0);
}

/* ========== 查询API实现 ========== */

const ErrorRecord *ErrorManager_GetHistory(uint16_t index) {
    if (index >= s_ctx.count) {
        return NULL;
    }
    
    // 计算实际索引（最新记录=0）
    uint16_t actual_index;
    if (s_ctx.count < ERROR_HISTORY_SIZE) {
        // 未满，从最新开始倒数
        actual_index = (s_ctx.count - 1 - index);
    } else {
        // 已满，从write_index开始倒数
        actual_index = (s_ctx.write_index + ERROR_HISTORY_SIZE - 1 - index) % ERROR_HISTORY_SIZE;
    }
    
    return &s_ctx.history[actual_index];
}

uint16_t ErrorManager_GetHistoryCount(void) {
    return s_ctx.count;
}

const ErrorStatistics *ErrorManager_GetStatistics(void) {
    return &s_ctx.stats;
}

bool ErrorManager_HasActiveFault(void) {
    return (s_ctx.active_faults != 0);
}

uint32_t ErrorManager_GetActiveFaults(void) {
    return s_ctx.active_faults;
}

bool ErrorManager_HasFaultInDomain(ErrorDomain domain) {
    return (s_ctx.active_faults & (1u << domain)) != 0;
}

/* ========== 控制API实现 ========== */

void ErrorManager_ClearAll(void) {
    s_ctx.write_index = 0;
    s_ctx.count = 0;
    s_ctx.active_faults = 0;
    memset(&s_ctx.stats, 0, sizeof(ErrorStatistics));
    LOGINFO("[ErrorMgr] All errors cleared");
}

void ErrorManager_ClearDomain(ErrorDomain domain) {
    s_ctx.active_faults &= ~(1u << domain);
}

void ErrorManager_ClearActiveFaults(void) {
    s_ctx.active_faults = 0;
    LOGINFO("[ErrorMgr] Active faults cleared");
}

/* ========== 回调API实现 ========== */

bool ErrorManager_RegisterCallback(ErrorCallback callback) {
    if (callback == NULL || s_ctx.callback_count >= ERROR_MAX_CALLBACKS) {
        return false;
    }
    
    // 检查是否已注册
    for (uint8_t i = 0; i < s_ctx.callback_count; i++) {
        if (s_ctx.callbacks[i] == callback) {
            return false; // 已存在
        }
    }
    
    s_ctx.callbacks[s_ctx.callback_count++] = callback;
    return true;
}

bool ErrorManager_UnregisterCallback(ErrorCallback callback) {
    for (uint8_t i = 0; i < s_ctx.callback_count; i++) {
        if (s_ctx.callbacks[i] == callback) {
            // 移除并压缩数组
            for (uint8_t j = i; j < s_ctx.callback_count - 1; j++) {
                s_ctx.callbacks[j] = s_ctx.callbacks[j + 1];
            }
            s_ctx.callback_count--;
            return true;
        }
    }
    return false;
}

/* ========== 工具API实现 ========== */

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

/* ========== 内部函数实现 ========== */

static void HandleErrorBySeverity(uint32_t error_code, ErrorSeverity severity) {
    switch (severity) {
        case ERROR_SEVERITY_INFO:
            // 仅记录，无需处理
            break;
            
        case ERROR_SEVERITY_WARNING:
            // 可选：LED指示、降低输出等
            break;
            
        case ERROR_SEVERITY_MINOR:
            // 降额运行（由应用层处理）
            break;
            
        case ERROR_SEVERITY_MAJOR:
            // 停机（由状态机处理）
            break;
            
        case ERROR_SEVERITY_CRITICAL:
            // 系统复位
            __disable_irq();
            LOGERROR("[ErrorMgr] CRITICAL ERROR 0x%08lX - System will reset", 
                     (unsigned long)error_code);
            // 等待看门狗复位或手动复位
            while(1) {
                // 死循环
            }
            break;
    }
}

static void InvokeCallbacks(const ErrorRecord *record) {
    for (uint8_t i = 0; i < s_ctx.callback_count; i++) {
        if (s_ctx.callbacks[i] != NULL) {
            s_ctx.callbacks[i](record);
        }
    }
}

static uint32_t GetSystemTick(void) {
    // 使用HAL提供的系统滴答
    extern uint32_t HAL_GetTick(void);
    return HAL_GetTick();
}
