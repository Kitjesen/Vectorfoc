#ifndef TEST_APP_INIT_BINDING_MOCKS_BSP_LOG_H
#define TEST_APP_INIT_BINDING_MOCKS_BSP_LOG_H
typedef struct { int unused; } UART_HandleTypeDef;
extern UART_HandleTypeDef HW_UART_DEBUG;
void LogInit(UART_HandleTypeDef *log_config);
#endif