#ifndef COMM_TEST_BOARD_CYCLE_COUNTER_H
#define COMM_TEST_BOARD_CYCLE_COUNTER_H
#include <stdint.h>
typedef struct { uint32_t CYCCNT; } CommTestDwt;
extern CommTestDwt *DWT;
#endif
