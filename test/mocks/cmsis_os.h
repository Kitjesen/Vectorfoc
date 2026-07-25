#ifndef TEST_MOCK_CMSIS_OS_H
#define TEST_MOCK_CMSIS_OS_H

#include "FreeRTOS.h"
#include "task.h"

typedef TaskHandle_t osThreadId;
typedef void (*os_pthread)(void const *argument);

typedef enum {
  osPriorityNormal = 0,
  osPriorityAboveNormal = 1
} osPriority;

typedef struct {
  const char *name;
  os_pthread pthread;
  osPriority tpriority;
  uint32_t instances;
  uint32_t stacksize;
  StackType_t *buffer;
  StaticTask_t *controlblock;
} osThreadDef_t;

#define osThreadStaticDef(name, thread, priority, instances, stacksz, buffer, control) \
  const osThreadDef_t os_thread_def_##name = {                                        \
      #name, (thread), (priority), (instances), (stacksz), (buffer), (control)}

#define osThread(name) (&os_thread_def_##name)

osThreadId osThreadCreate(const osThreadDef_t *thread_def, void *argument);

#endif /* TEST_MOCK_CMSIS_OS_H */
