#ifndef TEST_MOCK_TASK_H
#define TEST_MOCK_TASK_H

#include "FreeRTOS.h"

typedef void *TaskHandle_t;
typedef TaskHandle_t xTaskHandle;

#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING     2
#define taskSCHEDULER_SUSPENDED   0

BaseType_t xTaskGetSchedulerState(void);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask);

#endif /* TEST_MOCK_TASK_H */
