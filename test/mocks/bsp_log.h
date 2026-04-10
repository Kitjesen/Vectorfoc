// Copyright 2024-2026 VectorFOC Contributors
// SPDX-License-Identifier: Apache-2.0
/**
 * @file bsp_log.h (TEST_ENV stub)
 * @brief 宿主机测试环境下的 bsp_log 桩，将所有日志输出重定向到 printf。
 */
#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* 在 TEST_ENV 下不引入 bsp_usart.h（含 HAL），改用 mock */
#ifdef TEST_ENV
#include "mock_hal_types.h"  /* 提供 __disable_irq 等 CMSIS intrinsics */
typedef void *UART_HandleTypeDef;
#else
#include "bsp_usart.h"
#endif

typedef enum { LOG_DEBUG = 0, LOG_INFO, LOG_WARNING, LOG_ERROR } LOG_LEVEL;

static inline void LogInit(UART_HandleTypeDef *cfg) { (void)cfg; }

static inline void LOG_PROTO(const char *fmt, LOG_LEVEL level,
                              const char *file, int line,
                              const char *func, ...) {
    /* In TEST_ENV, suppress all log output to avoid va_list issues */
    (void)fmt; (void)level; (void)file; (void)line; (void)func;
}

#define LOGDEBUG(fmt, ...)   LOG_PROTO(fmt, LOG_DEBUG,   __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGINFO(fmt, ...)    LOG_PROTO(fmt, LOG_INFO,    __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGWARNING(fmt, ...) LOG_PROTO(fmt, LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGERROR(fmt, ...)   LOG_PROTO(fmt, LOG_ERROR,   __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#endif /* BSP_LOG_H */
