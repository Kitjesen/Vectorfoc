/********************************************************************************
 * @file        : log.c
 * @author      : INOVXIO
 * @brief       : ，init、output。
 * @version     : V1.0
 * @date        : 2025 - 1 - 11
 *
 * @details:
 *  - phase， `LogInit` init，phase。
 *  - `LOG_PROTO` output，（ `LOG_DEBUG`、`LOG_INFO`、`LOG_WARNING`、`LOG_ERROR`），
 *    ，DMAmode。
 *  - ， `LOGDEBUG`、`LOGINFO`、`LOGWARNING`  `LOGERROR` output。
 *
 * @note:
 *  - ，phasedriverconfiginit， `log_usart_instance` phaseconfig。
 *  -  `LOG_BUFFER_SIZE` ，set1024。，，。
 *  -  `USARTSend` ，DMAmode。error，errorposition，
 *    actualerror。
 *
 * @history:
 *  V1.0:
 *    - ，。、，。
 *
 * Copyright (c) 2025 INOVXIO. All rights reserved.
 ********************************************************************************/
#include "bsp_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static USARTInstance *log_usart_instance;
/**
 * @brief          LOG
 * @param[in]      log_config: LOGconfig
 * @retval         LOG
 */
void LogInit(UART_HandleTypeDef *log_config)
{
  USART_Init_Config_s config;
  config.usart_handle = log_config;
  log_usart_instance = USARTRegister(&config);
}
/**
 * @brief          output
 *
 * @param[in]      fmt:
 * @param[in]      level:
 * @param[in]      file:
 * @param[in]      line:
 * @param[in]      func:
 *
 * @retval         note
 */
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file,
               int line, const char *func, ...)
{
  char tmp[LOG_BUFFER_SIZE];
  char buf[LOG_BUFFER_SIZE];
  memset(tmp, 0, sizeof(tmp));
  memset(buf, 0, sizeof(buf));
  va_list args;
  va_start(args, func);
  vsnprintf(tmp, sizeof(tmp) - 1, fmt, args);
  va_end(args);
  switch (level)
  {
  case LOG_DEBUG:
    snprintf(buf, sizeof(buf), "[DEBUG] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  case LOG_INFO:
    snprintf(buf, sizeof(buf), "[INFO] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  case LOG_WARNING:
    snprintf(buf, sizeof(buf), "[WARN] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  case LOG_ERROR:
    snprintf(buf, sizeof(buf), "[ERROR] <%s> | <%d> | <%s>: %s\r\n", file, line, func, tmp);
    break;
  default:
    return;
  }
  USARTSend(log_usart_instance, (uint8_t *)buf, strlen(buf), USART_TRANSFER_BLOCKING);
}
