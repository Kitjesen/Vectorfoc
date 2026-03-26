/********************************************************************************
 * @file        : log.h
 * @author      : Hongsen Pang
 * @brief       :
 * ，init、output。
 * @version     : V1.0
 * @date        : 2025 - 1 - 11
 *
 * @details:
 *  - phase， `LogInit`
 * init，phase。
 *  - `LOG_PROTO` output，（
 * `LOG_DEBUG`、`LOG_INFO`、`LOG_WARNING`、`LOG_ERROR`），
 *    ，DMAmode。
 *  - ， `LOGDEBUG`、`LOGINFO`、`LOGWARNING`  `LOGERROR`
 * output。
 *
 * @note:
 *  - ，phasedriverconfiginit，
 * `log_usart_instance` phaseconfig。
 *  -  `LOG_BUFFER_SIZE`
 * ，set1024。，，。
 *  -  `USARTSend`
 * ，DMAmode。error，errorposition，
 *    actualerror。
 *
 * @history:
 *  V1.0:
 *    -
 * ，。、，。
 *
 * Copyright (c) 2026 Hongsen Pang. All rights reserved.
 ********************************************************************************/
#ifndef BSP_LOG_H
#define BSP_LOG_H
#include "bsp_usart.h"
#include <stdarg.h>
#include <stdbool.h>
#define LOG_BUFFER_SIZE 256 //
/**
 * @brief
 */
typedef enum {
  LOG_DEBUG = 0,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
} LOG_LEVEL;
/**
 * @brief init
 */
void LogInit(UART_HandleTypeDef *log_config);
/**
 * @brief output
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file, int line,
               const char *func, ...);
// debug level
#define LOGDEBUG(fmt, ...)                                                     \
  LOG_PROTO(fmt, LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// information level
#define LOGINFO(fmt, ...)                                                      \
  LOG_PROTO(fmt, LOG_INFO, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// warning level
#define LOGWARNING(fmt, ...)                                                   \
  LOG_PROTO(fmt, LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
// error level
#define LOGERROR(fmt, ...)                                                     \
  LOG_PROTO(fmt, LOG_ERROR, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#endif
