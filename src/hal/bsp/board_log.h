// Copyright 2024-2026 VectorFOC Contributors
// Licensed under the Apache License, Version 2.0.

#ifndef BSP_LOG_H
#define BSP_LOG_H

#include "main.h"

#ifndef LOG_BUFFER_SIZE
#define LOG_BUFFER_SIZE 256
#endif

typedef enum {
  LOG_DEBUG = 0,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
} LOG_LEVEL;

void LogInit(UART_HandleTypeDef *uart);
void LOG_PROTO(const char *fmt, LOG_LEVEL level, const char *file, int line,
               const char *func, ...);

#define LOGDEBUG(fmt, ...) \
  LOG_PROTO(fmt, LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGINFO(fmt, ...) \
  LOG_PROTO(fmt, LOG_INFO, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGWARNING(fmt, ...) \
  LOG_PROTO(fmt, LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define LOGERROR(fmt, ...) \
  LOG_PROTO(fmt, LOG_ERROR, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)

#endif /* BSP_LOG_H */
