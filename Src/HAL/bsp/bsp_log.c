/********************************************************************************
 * @file        : log.c
 * @author      : VectorFOC
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
 * Copyright (c) 2025 VectorFOC. All rights reserved.
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
  USART_Init_Config_s config = {
      .recv_buff_size = USART_RXBUFF_LIMIT,
      .usart_handle = log_config,
      .module_callback = NULL,
  };
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
  char buf[LOG_BUFFER_SIZE];
  if (fmt == NULL) {
    return;
  }

  const char *level_name;
  switch (level) {
  case LOG_DEBUG:
    level_name = "DEBUG";
    break;
  case LOG_INFO:
    level_name = "INFO";
    break;
  case LOG_WARNING:
    level_name = "WARN";
    break;
  case LOG_ERROR:
    level_name = "ERROR";
    break;
  default:
    return;
  }

  if (file == NULL) {
    file = "?";
  }
  if (func == NULL) {
    func = "?";
  }

  int prefix_len =
      snprintf(buf, sizeof(buf), "[%s] <%.48s> | <%d> | <%.32s>: ",
               level_name, file, line, func);
  if (prefix_len < 0) {
    return;
  }
  size_t used = (size_t)prefix_len;
  if (used >= sizeof(buf)) {
    used = sizeof(buf) - 1U;
  }

  va_list args;
  va_start(args, func);
  (void)vsnprintf(buf + used, sizeof(buf) - used, fmt, args);
  va_end(args);

  size_t len = strlen(buf);
  if (len + 2U < sizeof(buf)) {
    buf[len++] = '\r';
    buf[len++] = '\n';
    buf[len] = '\0';
  } else if (sizeof(buf) >= 3U) {
    buf[sizeof(buf) - 3U] = '\r';
    buf[sizeof(buf) - 2U] = '\n';
    buf[sizeof(buf) - 1U] = '\0';
  }

  if (log_usart_instance != NULL) {
    USARTSend(log_usart_instance, (uint8_t *)buf, (uint16_t)strlen(buf),
              USART_TRANSFER_BLOCKING);
  }
}
