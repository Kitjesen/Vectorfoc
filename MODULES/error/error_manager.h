/**
 * @file error_manager.h
 * @brief 统一错误管理器 - 全系统错误处理中心
 * @version 3.0
 * @date 2026-01-28
 * 
 * @details 功能：
 * - 统一错误记录和存储
 * - 错误历史追踪（循环缓冲区）
 * - 错误级别分类响应
 * - 错误统计和查询
 * - 错误回调机制
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

/* ========== 配置参数（可在error_config.h中覆盖） ========== */
#ifndef ERROR_HISTORY_SIZE
#define ERROR_HISTORY_SIZE 32  /**< 错误历史记录容量 */
#endif

#ifndef ERROR_MAX_CALLBACKS
#define ERROR_MAX_CALLBACKS 4  /**< 最大回调函数数量 */
#endif

/* ========== 错误统计结构 ========== */
typedef struct {
    uint32_t total_count;      /**< 总错误计数 */
    uint32_t info_count;       /**< 信息级别计数 */
    uint32_t warning_count;    /**< 警告级别计数 */
    uint32_t minor_count;      /**< 轻微故障计数 */
    uint32_t major_count;      /**< 严重故障计数 */
    uint32_t critical_count;   /**< 致命故障计数 */
    uint32_t last_error_time;  /**< 最后错误时间 */
} ErrorStatistics;

/* ========== 核心API ========== */

/**
 * @brief 初始化错误管理器
 * @note 必须在使用前调用
 */
void ErrorManager_Init(void);

/**
 * @brief 报告错误（完整版，带文件行号）
 * @param error_code 错误码
 * @param message 错误消息（可为NULL）
 * @param file 源文件名
 * @param line 源文件行号
 */
void ErrorManager_ReportFull(uint32_t error_code, const char *message, 
                             const char *file, uint32_t line);

/**
 * @brief 报告错误（简化版）
 * @param error_code 错误码
 * @param message 错误消息（可为NULL）
 */
void ErrorManager_Report(uint32_t error_code, const char *message);

/**
 * @brief 报告错误宏（自动填充文件和行号）
 */
#define ERROR_REPORT(code, msg) \
    ErrorManager_ReportFull(code, msg, __FILE__, __LINE__)

/**
 * @brief 报告错误宏（无消息版）
 */
#define ERROR_REPORT_CODE(code) \
    ErrorManager_ReportFull(code, NULL, __FILE__, __LINE__)

/* ========== 查询API ========== */

/**
 * @brief 获取错误历史记录
 * @param index 记录索引（0=最新，越大越旧）
 * @return 错误记录指针，NULL表示无效索引
 */
const ErrorRecord *ErrorManager_GetHistory(uint16_t index);

/**
 * @brief 获取历史记录数量
 * @return 当前存储的记录数量
 */
uint16_t ErrorManager_GetHistoryCount(void);

/**
 * @brief 获取错误统计信息
 * @return 统计信息指针
 */
const ErrorStatistics *ErrorManager_GetStatistics(void);

/**
 * @brief 检查是否有激活的故障
 * @return true=有故障，false=无故障
 */
bool ErrorManager_HasActiveFault(void);

/**
 * @brief 获取当前激活的故障码（位掩码）
 * @return 故障码位掩码
 */
uint32_t ErrorManager_GetActiveFaults(void);

/**
 * @brief 检查特定域是否有故障
 * @param domain 错误域
 * @return true=有故障，false=无故障
 */
bool ErrorManager_HasFaultInDomain(ErrorDomain domain);

/* ========== 控制API ========== */

/**
 * @brief 清除所有错误记录
 */
void ErrorManager_ClearAll(void);

/**
 * @brief 清除特定域的错误
 * @param domain 错误域
 */
void ErrorManager_ClearDomain(ErrorDomain domain);

/**
 * @brief 清除激活的故障标志
 * @note 仅清除标志，不删除历史记录
 */
void ErrorManager_ClearActiveFaults(void);

/* ========== 回调API ========== */

/**
 * @brief 注册错误回调函数
 * @param callback 回调函数指针
 * @return true=成功，false=失败（已满）
 */
bool ErrorManager_RegisterCallback(ErrorCallback callback);

/**
 * @brief 注销错误回调函数
 * @param callback 回调函数指针
 * @return true=成功，false=失败（未找到）
 */
bool ErrorManager_UnregisterCallback(ErrorCallback callback);

/* ========== 工具API ========== */

/**
 * @brief 获取错误域名称字符串
 * @param domain 错误域
 * @return 域名称字符串
 */
const char *ErrorManager_GetDomainName(ErrorDomain domain);

/**
 * @brief 获取严重级别名称字符串
 * @param severity 严重级别
 * @return 级别名称字符串
 */
const char *ErrorManager_GetSeverityName(ErrorSeverity severity);

/**
 * @brief 格式化错误码为字符串
 * @param error_code 错误码
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return 写入的字符数
 */
int ErrorManager_FormatError(uint32_t error_code, char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_MANAGER_H */
