// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#include "board_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef *s_log_uart;
/**
 * @brief          LOG
 * @param[in]      log_config: LOGconfig
 * @retval         LOG
 */
void LogInit(UART_HandleTypeDef *log_config)
{
  s_log_uart = log_config;
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
  memset(buf, 0, sizeof(buf));
  const char *level_tag = NULL;
  const char *file_name = strrchr(file, '/');
  const char *windows_file_name = strrchr(file, '\\');
  size_t used = 0;

  if (windows_file_name != NULL &&
      (file_name == NULL || windows_file_name > file_name)) {
    file_name = windows_file_name;
  }
  file_name = (file_name != NULL) ? (file_name + 1) : file;

  switch (level)
  {
  case LOG_DEBUG:
    level_tag = "DEBUG";
    break;
  case LOG_INFO:
    level_tag = "INFO";
    break;
  case LOG_WARNING:
    level_tag = "WARN";
    break;
  case LOG_ERROR:
    level_tag = "ERROR";
    break;
  default:
    return;
  }

  int prefix_len = snprintf(buf, sizeof(buf), "[%s] <%.48s> | <%d> | <%.32s>: ",
                            level_tag, file_name, line, func);
  if (prefix_len < 0) {
    return;
  }

  used = (size_t)prefix_len;
  if (used >= sizeof(buf)) {
    used = sizeof(buf) - 1;
  }

  va_list args;
  va_start(args, func);
  (void)vsnprintf(buf + used, sizeof(buf) - used, fmt, args);
  va_end(args);

  used = strlen(buf);
  if (used + 2 < sizeof(buf)) {
    buf[used++] = '\r';
    buf[used++] = '\n';
    buf[used] = '\0';
  } else {
    buf[sizeof(buf) - 3] = '\r';
    buf[sizeof(buf) - 2] = '\n';
    buf[sizeof(buf) - 1] = '\0';
  }

  if (s_log_uart != NULL) {
    (void)HAL_UART_Transmit(s_log_uart, (uint8_t *)buf,
                            (uint16_t)strlen(buf), 100);
  }
}
