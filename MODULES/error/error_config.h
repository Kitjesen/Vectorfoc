/**
 * @file error_config.h
 * @brief 统一错误管理器配置
 */

#ifndef ERROR_CONFIG_H
#define ERROR_CONFIG_H

/* 启用统一错误管理器 */
#define USE_ERROR_MANAGER

/* 错误历史记录容量（可调整）*/
#ifndef ERROR_HISTORY_SIZE
#define ERROR_HISTORY_SIZE 32
#endif

/* 最大回调函数数量 */
#ifndef ERROR_MAX_CALLBACKS
#define ERROR_MAX_CALLBACKS 4
#endif

#endif /* ERROR_CONFIG_H */
