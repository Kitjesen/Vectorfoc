#ifndef TEST_MOCK_FREERTOS_H
#define TEST_MOCK_FREERTOS_H

#include <stdint.h>

typedef uint32_t StackType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;

typedef struct {
  uint32_t reserved;
} StaticTask_t;

#define configMINIMAL_STACK_SIZE 128U
#define configSUPPORT_STATIC_ALLOCATION 1

#ifndef __weak
#define __weak
#endif

#endif /* TEST_MOCK_FREERTOS_H */
