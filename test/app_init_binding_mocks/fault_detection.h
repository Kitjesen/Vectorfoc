#ifndef TEST_APP_INIT_BINDING_MOCKS_FAULT_DETECTION_H
#define TEST_APP_INIT_BINDING_MOCKS_FAULT_DETECTION_H
#include "fault_def.h"
void Detection_Init(const DetectionConfig *config);
uint32_t Detection_Check_Slow(void *motor);
#endif